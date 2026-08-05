#include "task_remote.h"

#include "app_config.h"
#include "driver_uart.h"
#include "main.h"
#include "usart.h"

#include "cmsis_os2.h"

#include <string.h>

#define REMOTE_TASK_FLAG_RX 0x00000001UL
#define REMOTE_TASK_FLAG_ERROR 0x00000002UL

typedef struct
{
    uint8_t frame[DR16_FRAME_LEN];
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

static uint8_t dr16DmaBuffer[DR16_FRAME_LEN]
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

    if (copyLength > DR16_FRAME_LEN)
    {
        copyLength = DR16_FRAME_LEN;
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

    if (pending == NULL)
    {
        return 0U;
    }

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
    if ((remote == NULL) || (state == NULL) || (remoteStateMutex == NULL))
    {
        return;
    }
    if (osMutexAcquire(remoteStateMutex, osWaitForever) != osOK)
    {
        return;
    }

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
    dr16_data_t parsed;
    Remote_t converted;

    state->lastRxSize = pending->size;
    memcpy(state->rawFrame, pending->frame, sizeof(state->rawFrame));
    if (pending->size != DR16_FRAME_LEN)
    {
        state->invalidSizeCount++;
        if (state->online == 0U)
        {
            state->syncFrameCount = 0U;
        }
        return;
    }
    if (DR16_ParseFrame(pending->frame, &parsed) == 0U)
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
    if (parsed.dialValid == 0U)
    {
        state->invalidDialCount++;
    }

    state->keyPressed = 0U;
    state->keyReleased = 0U;
    if (state->online != 0U)
    {
        state->keyPressed = parsed.keyBits & (uint16_t)(~(*previousKeys));
        state->keyReleased = *previousKeys & (uint16_t)(~parsed.keyBits);
    }
    else
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

    state->dr16Data = parsed;
    DR16_MakeRemote(&parsed,
                    APP_DR16_DB,
                    APP_DR16_DIAL,
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
    *previousKeys = parsed.keyBits;
}

/** @brief 接收、校验并发布 DR16 数据，且不阻塞底盘控制任务。 */
static void RemoteTask(void *argument)
{
    task_remote_state_t state = {0};
    Remote_t remote = {0};
    task_remote_pending_t pending;
    uint16_t previousKeys = 0U;
    uint32_t rateTick = HAL_GetTick();
    uint32_t rateFrameCount = 0U;

    (void)argument;
    memset(dr16DmaBuffer, 0, sizeof(dr16DmaBuffer));
    Driver_UART_Init(&huart5,
                     dr16DmaBuffer,
                     sizeof(dr16DmaBuffer),
                     Remote_Task_RxCallback,
                     Remote_Task_ErrorCallback);
    (void)Driver_UART_StartRx(&huart5);

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
            state.keyPressed = 0U;
            state.keyReleased = 0U;
            previousKeys = state.dr16Data.keyBits;
            memset(&remote, 0, sizeof(remote));
        }
        if (driverUart5Object.receiving == 0U)
        {
            (void)Driver_UART_StartRx(&huart5);
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
    if (remoteStateMutex == NULL)
    {
        return;
    }
    remoteTaskHandle = osThreadNew(RemoteTask, NULL, &remoteTaskAttributes);
}

void Remote_Task_Update(void)
{
    if (remoteStateMutex == NULL)
    {
        return;
    }
    if (osMutexAcquire(remoteStateMutex, osWaitForever) != osOK)
    {
        return;
    }

    Remote = remotePublished;
    (void)osMutexRelease(remoteStateMutex);
}
