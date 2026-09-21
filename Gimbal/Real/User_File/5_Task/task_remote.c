#include "task_remote.h"

#include "app_config.h"
#include "driver_uart.h"
#include "main.h"
#include "usart.h"

#include "cmsis_os2.h"

#include <string.h>

#define REMOTE_TASK_FLAG_RX 0x00000001UL
#define REMOTE_TASK_FLAG_ERROR 0x00000002UL

/*
 * 后端差异全部收在这一处别名，函数体保持单一形态。
 * REMOTE_FRAME_LEN 在 task_remote.h 里已按后端定义。
 * 这些是编译期别名、不是运行时接口层：零间接、零开销，换后端只改 app_config.h 一个宏。
 */
#if APP_REMOTE_BACKEND == APP_REMOTE_BACKEND_IA10B
#define Remote_Backend_ParseFrame IA10B_ParseFrame
#define Remote_Backend_MakeRemote IA10B_MakeRemote
#define REMOTE_DEADBAND APP_IA10B_DB
#define REMOTE_UART_BAUD IA10B_UART_BAUD
#define REMOTE_UART_WORDLENGTH IA10B_UART_WORDLENGTH
#define REMOTE_UART_PARITY IA10B_UART_PARITY
#define REMOTE_UART_STOPBITS IA10B_UART_STOPBITS
typedef ia10b_data_t remote_backend_data_t;
#else
#define Remote_Backend_ParseFrame DR16_ParseFrame
#define Remote_Backend_MakeRemote DR16_MakeRemote
#define REMOTE_DEADBAND APP_DR16_DB
#define REMOTE_UART_BAUD DR16_UART_BAUD
#define REMOTE_UART_WORDLENGTH DR16_UART_WORDLENGTH
#define REMOTE_UART_PARITY DR16_UART_PARITY
#define REMOTE_UART_STOPBITS DR16_UART_STOPBITS
typedef dr16_data_t remote_backend_data_t;
#endif

typedef struct
{
    uint8_t frame[REMOTE_FRAME_LEN];
    uint16_t size;
    uint32_t tick;
    volatile uint8_t ready;
} task_remote_pending_t;

static osThreadId_t remoteTaskHandle;
static osMutexId_t remoteStateMutex;
static const osMutexAttr_t remoteStateMutexAttributes = {
    .name = "RemoteStateMutex",
    .attr_bits = osMutexPrioInherit,
};
static const osThreadAttr_t remoteTaskAttributes = {
    .name = "RemoteTask",
    .stack_size = 512U * 4U,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static uint8_t remoteDmaBuffer[REMOTE_FRAME_LEN]
    __attribute__((section(".ram_d1_dma"), aligned(32)));
static task_remote_pending_t remotePending;
static volatile uint32_t remoteOverwriteCount;
static volatile uint32_t remoteLastError;
static Remote_t remotePublished;

Remote_t Remote;
task_remote_state_t remoteTaskDebugState;

/** @brief 在 DMA 重新启用前，将本次接收事件复制到任务邮箱。 */
static void Remote_Task_RxCallback(const uint8_t *data, uint16_t length)
{
    uint16_t copyLength = length;

    if (copyLength > REMOTE_FRAME_LEN)
    {
        copyLength = REMOTE_FRAME_LEN;
    }
    if (remotePending.ready != 0U)
    {
        remoteOverwriteCount++;
    }

    memset(remotePending.frame, 0, sizeof(remotePending.frame));
    if ((data != NULL) && (copyLength > 0U))
    {
        memcpy(remotePending.frame, data, copyLength);
    }
    remotePending.size = length;
    remotePending.tick = HAL_GetTick();
    __DMB();
    remotePending.ready = 1U;
    (void)osThreadFlagsSet(remoteTaskHandle, REMOTE_TASK_FLAG_RX);
}

/** @brief HAL 记录 UART 接收错误后唤醒遥控器任务。 */
static void Remote_Task_ErrorCallback(uint32_t errorCode)
{
    remoteLastError = errorCode;
    (void)osThreadFlagsSet(remoteTaskHandle, REMOTE_TASK_FLAG_ERROR);
}

/** @brief 在临界区内取出最新的中断邮箱内容。 */
static uint8_t Remote_Task_TakePending(task_remote_pending_t *pending)
{
    uint32_t interruptState;
    uint8_t hasPending = 0U;

    interruptState = __get_PRIMASK();
    __disable_irq();
    __DMB();
    if (remotePending.ready != 0U)
    {
        *pending = remotePending;
        remotePending.ready = 0U;
        hasPending = 1U;
    }
    if (interruptState == 0U)
    {
        __enable_irq();
    }
    return hasPending;
}

/** @brief 为底盘任务和Watch同时发布控制输入与协议诊断快照。 */
static void Remote_Task_Publish(const Remote_t *remote,
                                const task_remote_state_t *state)
{
    (void)osMutexAcquire(remoteStateMutex, osWaitForever);

    remotePublished = *remote;
    remoteTaskDebugState = *state;
    (void)osMutexRelease(remoteStateMutex);
}

/** @brief 解析一个待处理事件并更新遥控器上线同步状态。 */
static void Remote_Task_ProcessFrame(task_remote_state_t *state,
                                     Remote_t *remote,
                                     const task_remote_pending_t *pending,
                                     uint16_t *previousKeys)
{
    remote_backend_data_t parsed;
    Remote_t converted;

    state->lastRxSize = pending->size;
    memcpy(state->rawFrame, pending->frame, sizeof(state->rawFrame));
    if (pending->size != REMOTE_FRAME_LEN)
    {
        state->invalidSizeCount++;
        if (state->online == 0U)
        {
            state->syncFrameCount = 0U;
        }
        return;
    }
    if (Remote_Backend_ParseFrame(pending->frame, &parsed) == 0U)
    {
        state->invalidFrameCount++;
        if (state->online == 0U)
        {
            state->syncFrameCount = 0U;
        }
        return;
    }

    state->validFrameCount++;
    state->lastValidTick = pending->tick;
#if APP_REMOTE_BACKEND == APP_REMOTE_BACKEND_DR16
    if (parsed.dialValid == 0U)
    {
        state->invalidDialCount++;
    }
#endif

    /*
     * ⚠ 键鼠沿是 DR16 独有的，但它必须排在同步计数【之前】：原来两者是 if/else
     * 互斥的，拆开后若让同步块先跑，上线那一帧 online 刚置 1、previousKeys 还是
     * 陈旧值，会凭空报一次按键按下。
     */
#if APP_REMOTE_BACKEND == APP_REMOTE_BACKEND_DR16
    state->keyPressed = 0U;
    state->keyReleased = 0U;
    if (state->online != 0U)
    {
        state->keyPressed = parsed.keyBits & (uint16_t)(~(*previousKeys));
        state->keyReleased = *previousKeys & (uint16_t)(~parsed.keyBits);
    }
#else
    (void)previousKeys;   /* i-BUS 没有键鼠通道，这一路不消费它 */
#endif
    if (state->online == 0U)
    {
        if (state->syncFrameCount < APP_REMOTE_SYNC_FRAMES)
        {
            state->syncFrameCount++;
        }
        if (state->syncFrameCount >= APP_REMOTE_SYNC_FRAMES)
        {
            state->online = 1U;
        }
    }

    state->backendData = parsed;
    Remote_Backend_MakeRemote(&parsed,
                              REMOTE_DEADBAND,
                              &converted);
    if (state->online != 0U)
    {
        converted.online = 1U;
        *remote = converted;
    }
    else
    {
        memset(remote, 0, sizeof(*remote));
    }
#if APP_REMOTE_BACKEND == APP_REMOTE_BACKEND_DR16
    *previousKeys = parsed.keyBits;
#endif
}

/** @brief 接收、校验并发布遥控数据，且不阻塞底盘控制任务。 */
static void Remote_Task_Entry(void *argument)
{
    task_remote_state_t state = {0};
    Remote_t remote = {0};
    task_remote_pending_t pending;
    uint16_t previousKeys = 0U;
    uint32_t rateTick = HAL_GetTick();
    uint32_t rateFrameCount = 0U;

    (void)argument;
    memset(remoteDmaBuffer, 0, sizeof(remoteDmaBuffer));
    /*
     * 按所选后端重新初始化 UART5。两种接收机挂同一路物理线，但 DBUS 是
     * 100000 8E1、i-BUS 是 115200 8N1（两者都是正常电平，不需要 RX 反相）。
     * 这里重配而不是改 Core/Src/usart.c，是因为那个文件由 CubeMX 生成，
     * 重新生成会被冲掉；DR16 后端填的就是 CubeMX 现有的值，对它是空操作。
     */
    huart5.Init.BaudRate = REMOTE_UART_BAUD;
    huart5.Init.WordLength = REMOTE_UART_WORDLENGTH;
    huart5.Init.Parity = REMOTE_UART_PARITY;
    huart5.Init.StopBits = REMOTE_UART_STOPBITS;
    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        Error_Handler();
    }
    Driver_UART_Init(&huart5,
                     remoteDmaBuffer,
                     sizeof(remoteDmaBuffer),
                     Remote_Task_RxCallback,
                     Remote_Task_ErrorCallback);
    Driver_UART_StartRx(&huart5);

    for (;;)
    {
        uint32_t nowTick;

        (void)osThreadFlagsWait(REMOTE_TASK_FLAG_RX | REMOTE_TASK_FLAG_ERROR,
                                osFlagsWaitAny,
                                APP_REMOTE_WAIT_TICKS);
        while (Remote_Task_TakePending(&pending) != 0U)
        {
            Remote_Task_ProcessFrame(&state,
                                     &remote,
                                     &pending,
                                     &previousKeys);
        }

        nowTick = HAL_GetTick();
        if ((state.online != 0U) &&
            ((nowTick - state.lastValidTick) > APP_REMOTE_TIMEOUT_TICKS))
        {
            state.online = 0U;
            state.syncFrameCount = 0U;
#if APP_REMOTE_BACKEND == APP_REMOTE_BACKEND_DR16
            state.keyPressed = 0U;
            state.keyReleased = 0U;
            /* 掉线时把基准对齐到最后一帧，重新上线不会报出一次假的按键沿。 */
            previousKeys = state.backendData.keyBits;
#endif
            memset(&remote, 0, sizeof(remote));
        }
        if (driverUart5Object.receiving == 0U)
        {
            Driver_UART_StartRx(&huart5);
        }

        state.rxEventCount = driverUart5Object.rxEventCount;
        state.overwriteCount = remoteOverwriteCount;
        state.uartErrorCount = driverUart5Object.errorCount;
        state.restartErrorCount = driverUart5Object.restartErrorCount;
        state.lastUartError = remoteLastError;
        if ((nowTick - rateTick) >= 1000U)
        {
            uint32_t elapsedTick = nowTick - rateTick;
            uint32_t validDelta = state.validFrameCount - rateFrameCount;

            state.validFrameRateHz =
                (validDelta * 1000U) / elapsedTick;
            rateTick = nowTick;
            rateFrameCount = state.validFrameCount;
        }
        Remote_Task_Publish(&remote, &state);
    }
}

void Remote_Task_Init(void)
{
    memset(&remotePending, 0, sizeof(remotePending));
    memset(&remotePublished, 0, sizeof(remotePublished));
    memset(&Remote, 0, sizeof(Remote));
    memset(&remoteTaskDebugState, 0, sizeof(remoteTaskDebugState));
    remoteOverwriteCount = 0U;
    remoteLastError = 0U;

    remoteStateMutex = osMutexNew(&remoteStateMutexAttributes);
    remoteTaskHandle = osThreadNew(Remote_Task_Entry, NULL, &remoteTaskAttributes);
}

void Remote_Task_Update(void)
{
    (void)osMutexAcquire(remoteStateMutex, osWaitForever);

    Remote = remotePublished;
    (void)osMutexRelease(remoteStateMutex);
}
