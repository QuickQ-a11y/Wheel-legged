#include "task_can.h"

#include "driver_fdcan.h"

#include "cmsis_os2.h"

#include <string.h>

#define CAN_TASK_FLAG_RX 0x00000001UL
#define CAN_TASK_FLAG_TX 0x00000002UL

typedef struct
{
    FDCAN_HandleTypeDef *handle;                         /* 目标 FDCAN 外设。 */
    uint32_t identifier;                                 /* 标准帧 ID。 */
    uint8_t length;                                      /* 数据长度。 */
    uint8_t data[APP_CAN_DATA_MAX_BYTES];        /* 待发送数据。 */
} task_can_tx_frame_t;

static osThreadId_t canTaskHandle;
static osMutexId_t canTxMutex;
static osMessageQueueId_t canRxQueue;

static task_can_tx_frame_t canTxFrames[APP_CAN_TX_CAP];
static volatile uint32_t canTxErrorCount;
static volatile uint32_t canRxOverflowCount;

static const osThreadAttr_t canTaskAttributes = {
    .name = "CanTask",
    .stack_size = 512U * 4U,
    .priority = (osPriority_t)osPriorityNormal,
};

static const osMutexAttr_t canTxMutexAttributes = {
    .name = "CanTxMutex",
};

/**
 * @brief 将 HAL FDCAN 句柄转换为应用层总线枚举。
 */
static app_can_bus_t CAN_Task_GetBus(FDCAN_HandleTypeDef *handle)
{
    if (handle == &hfdcan1)
    {
        return APP_CAN_BUS_FDCAN1;
    }

    if (handle == &hfdcan2)
    {
        return APP_CAN_BUS_FDCAN2;
    }

    return APP_CAN_BUS_UNKNOWN;
}

static int32_t CAN_Task_FindTxFrame(FDCAN_HandleTypeDef *handle,
                                    uint32_t identifier)
{
    uint32_t index;

    for (index = 0U; index < APP_CAN_TX_CAP; index++)
    {
        if ((canTxFrames[index].handle != NULL) &&
            (canTxFrames[index].handle == handle) &&
            (canTxFrames[index].identifier == identifier))
        {
            return (int32_t)index;
        }
    }

    return -1;
}

static int32_t CAN_Task_FindFreeTxFrame(void)
{
    uint32_t index;

    for (index = 0U; index < APP_CAN_TX_CAP; index++)
    {
        if (canTxFrames[index].handle == NULL)
        {
            return (int32_t)index;
        }
    }

    return -1;
}

/**
 * @brief FDCAN 驱动接收回调，将中断报文转存到任务队列。
 */
static void CAN_Task_RxCallback(FDCAN_HandleTypeDef *handle,
                                const driver_fdcan_rx_frame_t *frame)
{
    task_can_rx_message_t message = {0};
    uint32_t flags;

    message.bus = CAN_Task_GetBus(handle);
    message.identifier = frame->header.Identifier;
    message.length = Driver_FDCAN_DlcToLength(frame->header.DataLength);
    if (message.length > APP_CAN_DATA_MAX_BYTES)
    {
        message.length = APP_CAN_DATA_MAX_BYTES;
    }

    memcpy(message.data, frame->data, message.length);

    if (osMessageQueuePut(canRxQueue, &message, 0U, 0U) != osOK)
    {
        canRxOverflowCount++;
        return;
    }

    flags = osThreadFlagsSet(canTaskHandle, CAN_TASK_FLAG_RX);
    if ((flags & (uint32_t)osFlagsError) != 0U)
    {
        canRxOverflowCount++;
    }
}

/**
 * @brief 发送所有已缓存的最新命令帧。
 */
static void CAN_Task_SendFrames(void)
{
    uint32_t index;

    for (index = 0U; index < APP_CAN_TX_CAP; index++)
    {
        task_can_tx_frame_t frame = {0};

        (void)osMutexAcquire(canTxMutex, osWaitForever);

        frame = canTxFrames[index];
        (void)osMutexRelease(canTxMutex);

        if (frame.handle == NULL)
        {
            continue;
        }

        Driver_FDCAN_SendData(frame.handle, frame.identifier, frame.data, frame.length);
    }
}

/**
 * @brief 分发接收队列中的全部报文。
 */
static void CAN_Task_DispatchRx(void)
{
    task_can_rx_message_t message;

    while (osMessageQueueGet(canRxQueue, &message, NULL, 0U) == osOK)
    {
        CAN_Task_RxMessageCallback(&message);
    }
}

static void CAN_Task_Entry(void *argument)
{
    uint32_t wakeTick = osKernelGetTickCount();

    (void)argument;

    Driver_FDCAN_Init(&hfdcan1, CAN_Task_RxCallback);
    Driver_FDCAN_Init(&hfdcan2, CAN_Task_RxCallback);

    for (;;)
    {
        uint32_t flags = osThreadFlagsWait(CAN_TASK_FLAG_RX | CAN_TASK_FLAG_TX,
                                           osFlagsWaitAny,
                                           APP_CAN_PERIOD_TICKS);

        if (flags == (uint32_t)osFlagsErrorTimeout)
        {
            CAN_Task_SendFrames();
        }
        else if ((flags & (uint32_t)osFlagsError) != 0U)
        {
            canTxErrorCount++;
        }
        else
        {
            if ((flags & CAN_TASK_FLAG_RX) != 0U)
            {
                CAN_Task_DispatchRx();
            }

            if ((flags & CAN_TASK_FLAG_TX) != 0U)
            {
                CAN_Task_SendFrames();
            }
        }

        wakeTick += APP_CAN_PERIOD_TICKS;
        (void)osDelayUntil(wakeTick);
    }
}

void CAN_Task_Init(void)
{
    canTxMutex = osMutexNew(&canTxMutexAttributes);
    canRxQueue = osMessageQueueNew(APP_CAN_RX_QUEUE_LEN,
                                   sizeof(task_can_rx_message_t),
                                   NULL);

    canTaskHandle = osThreadNew(CAN_Task_Entry, NULL, &canTaskAttributes);
}

void CAN_Task_SetDjiCurrent(const int16_t current[APP_WHEEL_COUNT])
{
    uint8_t data[APP_DJI_TX_LEN] = {0};

    data[0] = (uint8_t)((uint16_t)current[0] >> 8U);
    data[1] = (uint8_t)((uint16_t)current[0]);
    data[2] = (uint8_t)((uint16_t)current[1] >> 8U);
    data[3] = (uint8_t)((uint16_t)current[1]);

    CAN_Task_UpdateTxFrame(&hfdcan2,
                           APP_DJI_TX_ID,
                           data,
                           APP_DJI_TX_LEN);
}

void CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle,
                            uint32_t identifier,
                            const uint8_t *data,
                            uint8_t length)
{
    int32_t frameIndex;

    if (length > APP_CAN_DATA_MAX_BYTES)
    {
        canTxErrorCount++;
        return;
    }

    (void)osMutexAcquire(canTxMutex, osWaitForever);

    frameIndex = CAN_Task_FindTxFrame(handle, identifier);
    if (frameIndex < 0)
    {
        frameIndex = CAN_Task_FindFreeTxFrame();
    }

    if (frameIndex < 0)
    {
        (void)osMutexRelease(canTxMutex);
        canTxErrorCount++;
        return;
    }

    canTxFrames[frameIndex].handle = handle;
    canTxFrames[frameIndex].identifier = identifier;
    canTxFrames[frameIndex].length = length;
    memset(canTxFrames[frameIndex].data, 0, sizeof(canTxFrames[frameIndex].data));
    memcpy(canTxFrames[frameIndex].data, data, length);

    (void)osMutexRelease(canTxMutex);
}

void CAN_Task_RequestTx(void)
{
    uint32_t flags;

    flags = osThreadFlagsSet(canTaskHandle, CAN_TASK_FLAG_TX);
    if ((flags & (uint32_t)osFlagsError) != 0U)
    {
        canTxErrorCount++;
    }
}

uint32_t CAN_Task_GetTxErrorCount(void)
{
    return canTxErrorCount;
}

__weak void CAN_Task_RxMessageCallback(const task_can_rx_message_t *message)
{
    (void)message;
}
