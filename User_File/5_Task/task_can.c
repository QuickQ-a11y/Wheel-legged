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
    uint8_t enabled;                                     /* 是否参与周期发送。 */
    uint8_t used;                                        /* 槽位是否已被占用。 */
    uint8_t data[APP_CONFIG_CAN_MAX_DATA_LENGTH];        /* 待发送数据。 */
} task_can_tx_frame_t;

static osThreadId_t canTaskHandle;
static osMutexId_t canTxMutex;
static osMessageQueueId_t canRxQueue;

static task_can_tx_frame_t canTxFrames[APP_CONFIG_CAN_TX_FRAME_CAPACITY];
static volatile uint32_t canTxBusyCount;
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

static uint8_t CAN_Task_IsValidHandle(FDCAN_HandleTypeDef *handle)
{
    return ((handle == &hfdcan1) || (handle == &hfdcan2)) ? 1U : 0U;
}

static int32_t CAN_Task_FindTxFrame(FDCAN_HandleTypeDef *handle,
                                    uint32_t identifier)
{
    uint32_t index;

    for (index = 0U; index < APP_CONFIG_CAN_TX_FRAME_CAPACITY; index++)
    {
        if ((canTxFrames[index].used != 0U) &&
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

    for (index = 0U; index < APP_CONFIG_CAN_TX_FRAME_CAPACITY; index++)
    {
        if (canTxFrames[index].used == 0U)
        {
            return (int32_t)index;
        }
    }

    return -1;
}

/**
 * @brief 从发送表复制一帧，避免发送时长期持有互斥锁。
 */
static app_status_t CAN_Task_CopyTxFrame(uint32_t index,
                                         task_can_tx_frame_t *frame)
{
    if ((index >= APP_CONFIG_CAN_TX_FRAME_CAPACITY) || (frame == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    if (osMutexAcquire(canTxMutex, osWaitForever) != osOK)
    {
        return APP_STATUS_BUSY;
    }

    *frame = canTxFrames[index];
    (void)osMutexRelease(canTxMutex);

    return APP_STATUS_OK;
}

/**
 * @brief FDCAN 驱动接收回调，将中断报文转存到任务队列。
 */
static void CAN_Task_RxCallback(FDCAN_HandleTypeDef *handle,
                                const driver_fdcan_rx_frame_t *frame)
{
    task_can_rx_message_t message = {0};
    uint32_t flags;

    if ((frame == NULL) || (canRxQueue == NULL))
    {
        return;
    }

    message.bus = CAN_Task_GetBus(handle);
    message.identifier = frame->header.Identifier;
    message.length = Driver_FDCAN_DlcToLength(frame->header.DataLength);
    if (message.length > APP_CONFIG_CAN_MAX_DATA_LENGTH)
    {
        message.length = APP_CONFIG_CAN_MAX_DATA_LENGTH;
    }

    memcpy(message.data, frame->data, message.length);

    if (osMessageQueuePut(canRxQueue, &message, 0U, 0U) != osOK)
    {
        canRxOverflowCount++;
        return;
    }

    if (canTaskHandle != NULL)
    {
        flags = osThreadFlagsSet(canTaskHandle, CAN_TASK_FLAG_RX);
        if ((flags & (uint32_t)osFlagsError) != 0U)
        {
            canRxOverflowCount++;
        }
    }
}

/**
 * @brief 发送所有已启用的缓存帧。
 */
static void CAN_Task_SendFrames(void)
{
    uint32_t index;

    for (index = 0U; index < APP_CONFIG_CAN_TX_FRAME_CAPACITY; index++)
    {
        task_can_tx_frame_t frame = {0};
        app_status_t status;

        if (CAN_Task_CopyTxFrame(index, &frame) != APP_STATUS_OK)
        {
            canTxErrorCount++;
            continue;
        }

        if ((frame.used == 0U) || (frame.enabled == 0U))
        {
            continue;
        }

        status = Driver_FDCAN_SendData(frame.handle, frame.identifier, frame.data, frame.length);
        if (status == APP_STATUS_BUSY)
        {
            canTxBusyCount++;
        }
        else if (status != APP_STATUS_OK)
        {
            canTxErrorCount++;
        }
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

static void CanTask(void *argument)
{
    uint32_t wakeTick = osKernelGetTickCount();

    (void)argument;

    if (Driver_FDCAN_Init(&hfdcan1, CAN_Task_RxCallback) != APP_STATUS_OK)
    {
        canTxErrorCount++;
    }

    if (Driver_FDCAN_Init(&hfdcan2, CAN_Task_RxCallback) != APP_STATUS_OK)
    {
        canTxErrorCount++;
    }

    for (;;)
    {
        uint32_t flags = osThreadFlagsWait(CAN_TASK_FLAG_RX | CAN_TASK_FLAG_TX,
                                           osFlagsWaitAny,
                                           APP_CONFIG_CAN_TASK_PERIOD_TICKS);

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

        wakeTick += APP_CONFIG_CAN_TASK_PERIOD_TICKS;
        (void)osDelayUntil(wakeTick);
    }
}

void CAN_Task_Init(void)
{
    canTxMutex = osMutexNew(&canTxMutexAttributes);
    canRxQueue = osMessageQueueNew(APP_CONFIG_CAN_RX_QUEUE_LENGTH,
                                   sizeof(task_can_rx_message_t),
                                   NULL);

    if ((canTxMutex == NULL) || (canRxQueue == NULL))
    {
        return;
    }

    canTaskHandle = osThreadNew(CanTask, NULL, &canTaskAttributes);
}

app_status_t CAN_Task_SetDjiCurrent(const int16_t current[APP_CONFIG_DJI_WHEEL_COUNT])
{
    uint8_t data[APP_CONFIG_DJI_COMMAND_LENGTH] = {0};

    if (current == NULL)
    {
        return APP_STATUS_INVALID_PARAM;
    }

    data[0] = (uint8_t)((uint16_t)current[0] >> 8U);
    data[1] = (uint8_t)((uint16_t)current[0]);
    data[2] = (uint8_t)((uint16_t)current[1] >> 8U);
    data[3] = (uint8_t)((uint16_t)current[1]);

    return CAN_Task_UpdateTxFrame(&hfdcan2,
                                  APP_CONFIG_DJI_CURRENT_TX_ID,
                                  data,
                                  APP_CONFIG_DJI_COMMAND_LENGTH);
}

app_status_t CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle,
                                    uint32_t identifier,
                                    const uint8_t *data,
                                    uint8_t length)
{
    int32_t frameIndex;

    if ((CAN_Task_IsValidHandle(handle) == 0U) ||
        (identifier > APP_CONFIG_CAN_STD_ID_MAX) ||
        (data == NULL) ||
        (length > APP_CONFIG_CAN_MAX_DATA_LENGTH) ||
        (canTxMutex == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    if (osMutexAcquire(canTxMutex, osWaitForever) != osOK)
    {
        return APP_STATUS_BUSY;
    }

    frameIndex = CAN_Task_FindTxFrame(handle, identifier);
    if (frameIndex < 0)
    {
        frameIndex = CAN_Task_FindFreeTxFrame();
    }

    if (frameIndex < 0)
    {
        (void)osMutexRelease(canTxMutex);
        return APP_STATUS_NO_RESOURCE;
    }

    canTxFrames[frameIndex].handle = handle;
    canTxFrames[frameIndex].identifier = identifier;
    canTxFrames[frameIndex].length = length;
    canTxFrames[frameIndex].enabled = 1U;
    canTxFrames[frameIndex].used = 1U;
    memset(canTxFrames[frameIndex].data, 0, sizeof(canTxFrames[frameIndex].data));
    memcpy(canTxFrames[frameIndex].data, data, length);

    (void)osMutexRelease(canTxMutex);

    return APP_STATUS_OK;
}

app_status_t CAN_Task_EnableTxFrame(FDCAN_HandleTypeDef *handle,
                                    uint32_t identifier,
                                    uint8_t enabled)
{
    int32_t frameIndex;

    if ((CAN_Task_IsValidHandle(handle) == 0U) || (canTxMutex == NULL))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    if (osMutexAcquire(canTxMutex, osWaitForever) != osOK)
    {
        return APP_STATUS_BUSY;
    }

    frameIndex = CAN_Task_FindTxFrame(handle, identifier);
    if (frameIndex < 0)
    {
        (void)osMutexRelease(canTxMutex);
        return APP_STATUS_NOT_READY;
    }

    canTxFrames[frameIndex].enabled = (enabled != 0U) ? 1U : 0U;
    (void)osMutexRelease(canTxMutex);

    return APP_STATUS_OK;
}

app_status_t CAN_Task_RequestTx(void)
{
    uint32_t flags;

    if (canTaskHandle == NULL)
    {
        return APP_STATUS_NOT_READY;
    }

    flags = osThreadFlagsSet(canTaskHandle, CAN_TASK_FLAG_TX);

    return ((flags & (uint32_t)osFlagsError) != 0U) ? APP_STATUS_ERROR : APP_STATUS_OK;
}

uint32_t CAN_Task_GetTxBusyCount(void)
{
    return canTxBusyCount;
}

uint32_t CAN_Task_GetTxErrorCount(void)
{
    return canTxErrorCount;
}

uint32_t CAN_Task_GetRxOverflowCount(void)
{
    return canRxOverflowCount;
}

__weak void CAN_Task_RxMessageCallback(const task_can_rx_message_t *message)
{
    (void)message;
}
