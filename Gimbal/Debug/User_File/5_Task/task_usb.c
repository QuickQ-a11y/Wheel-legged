#include "task_usb.h"

#include "app_config.h"
#include "driver_usb.h"
#include "main.h"
#include "task_imu.h"
#include "usbd_def.h"

#include "cmsis_os2.h"

#include <string.h>

#define USB_TASK_FLAG_RECEIVE 0x00000001UL
#define USB_TASK_FLAG_TRANSMIT 0x00000002UL
#define USB_TASK_RX_BUFFER_MASK (APP_USB_RX_BUF_SIZE - 1U)
#define USB_TASK_TX_BATCH_SIZE \
    (sizeof(usb_status_packet_t) * APP_USB_TX_BATCH)

static_assert((APP_USB_RX_BUF_SIZE & USB_TASK_RX_BUFFER_MASK) == 0U,
               "USB RX buffer size must be a power of two");

typedef struct
{
    usb_status_packet_t frames[APP_USB_TX_CAP];
    uint16_t writeIndex;
    uint16_t readIndex;
    uint16_t frameCount;
} task_usb_tx_queue_t;

static osThreadId_t usbTaskHandle;

static uint8_t usbReceiveBuffer[APP_USB_RX_BUF_SIZE];
static volatile uint16_t usbReceiveWriteIndex;
static volatile uint16_t usbReceiveReadIndex;

static task_usb_tx_queue_t usbTransmitQueue;
static uint8_t usbTransmitBuffer[USB_TASK_TX_BATCH_SIZE] __attribute__((aligned(4)));
static volatile uint16_t usbTransmitLength;
static volatile uint8_t usbTransmitActive;
static volatile uint16_t usbActiveFrameCount;
static uint8_t usbTransmitSequence;

uint16_t usbTaskTxCommandId = 0x0002U;
usb_status_payload_t usbTaskTxData = {
    .roll = 0.0f,
    .yaw = 0.0f,
    .pitch = 0.0f,
    .bulletSpeed = 0.0f, /* 暂无数据源，调试时在此处或 Live Watch 手动赋值。 */
    .heat = 0.0f,        /* 暂无数据源，调试时在此处或 Live Watch 手动赋值。 */
    .enemyColor = 0U,    /* 暂无数据源，调试时在此处或 Live Watch 手动赋值。 */
    .mode = 0U,          /* 暂无数据源，调试时在此处或 Live Watch 手动赋值。 */
};
task_usb_debug_t usbTaskDebugState;

static const osThreadAttr_t usbTaskAttributes = {
    .name = "UsbTask",
    .stack_size = 768U * 4U,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

/**
 * @brief 在 USB 中断中把新字节写入单生产者、单消费者环形缓冲。
 */
static void USB_Task_ReceiveCallback(const uint8_t *data, uint32_t length)
{
    uint32_t index;
    uint16_t nextWriteIndex;

    usbTaskDebugState.rxByteCount += length;

    for (index = 0U; index < length; index++)
    {
        nextWriteIndex = (usbReceiveWriteIndex + 1U) & USB_TASK_RX_BUFFER_MASK;
        if (nextWriteIndex == usbReceiveReadIndex)
        {
            usbTaskDebugState.rxOverflowCount += length - index;
            break;
        }

        usbReceiveBuffer[usbReceiveWriteIndex] = data[index];
        usbReceiveWriteIndex = nextWriteIndex;
    }

    (void)osThreadFlagsSet(usbTaskHandle, USB_TASK_FLAG_RECEIVE);
}

/**
 * @brief 释放当前 CDC IN 缓冲，并唤醒任务继续发送下一批数据。
 */
static void USB_Task_TransmitCallback(void)
{
    if (usbTransmitActive != 0U)
    {
        usbTaskDebugState.txCompleteCount += usbActiveFrameCount;
    }

    usbActiveFrameCount = 0U;
    usbTransmitActive = 0U;
    (void)osThreadFlagsSet(usbTaskHandle, USB_TASK_FLAG_TRANSMIT);
}

/**
 * @brief USB 断开时丢弃协议栈正在占用的旧批次，重新连接后从最新队列继续。
 */
static void USB_Task_DisconnectCallback(void)
{
    usbTaskDebugState.txDroppedCount += usbActiveFrameCount;
    usbActiveFrameCount = 0U;
    usbTransmitLength = 0U;
    usbTransmitActive = 0U;
    (void)osThreadFlagsSet(usbTaskHandle, USB_TASK_FLAG_TRANSMIT);
}

/**
 * @brief 分发 CRC 校验通过的协议帧。
 */
static void USB_Task_FrameReceived(uint16_t commandId,
                                   uint8_t sequence,
                                   const uint8_t *payload,
                                   uint16_t payloadLength)
{
    uint8_t sequenceDelta;

    if (usbTaskDebugState.rxFrameCount > 0U)
    {
        sequenceDelta = (uint8_t)(sequence - usbTaskDebugState.recentRxSeq);
        if (sequenceDelta == 0U)
        {
            usbTaskDebugState.rxDuplicateCount++;
        }
        else if (sequenceDelta <= 127U)
        {
            usbTaskDebugState.rxLostCount += sequenceDelta - 1U;
        }
        else
        {
            /* USB 不会重排数据，大跨度反向变化按序号源重置处理。 */
            usbTaskDebugState.rxSeqResetCount++;
        }
    }

    usbTaskDebugState.recentRxSeq = sequence;
    usbTaskDebugState.rxFrameCount++;

    if (payloadLength != sizeof(usb_control_payload_t))
    {
        usbTaskDebugState.rxOtherCount++;
        return;
    }

    USB_Protocol_UnpackControl(payload, &usbTaskDebugState.recentControl);
    usbTaskDebugState.recentControlCmdId = commandId;
    usbTaskDebugState.recentControlSeq = sequence;
    usbTaskDebugState.rxControlCount++;
}

/**
 * @brief 把接收环形缓冲中的数据分块送入协议解析器。
 */
static void USB_Task_ProcessReceivedData(void)
{
    uint8_t parseBuffer[128];
    uint16_t parseLength;

    while (usbReceiveReadIndex != usbReceiveWriteIndex)
    {
        parseLength = 0U;
        while ((usbReceiveReadIndex != usbReceiveWriteIndex) &&
               (parseLength < sizeof(parseBuffer)))
        {
            parseBuffer[parseLength] = usbReceiveBuffer[usbReceiveReadIndex];
            parseLength++;
            usbReceiveReadIndex =
                (usbReceiveReadIndex + 1U) & USB_TASK_RX_BUFFER_MASK;
        }

        USB_Protocol_Parse(&usbTaskDebugState.rxParser,
                           parseBuffer,
                           parseLength);
    }
}

/**
 * @brief 将状态帧写入固定容量发送队列，队列满时覆盖最旧帧。
 */
static void USB_Task_QueueFrame(const usb_status_packet_t *packet)
{
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();

    if (usbTransmitQueue.frameCount >= APP_USB_TX_CAP)
    {
        usbTransmitQueue.readIndex =
            (usbTransmitQueue.readIndex + 1U) % APP_USB_TX_CAP;
        usbTransmitQueue.frameCount--;
        usbTaskDebugState.txDroppedCount++;
    }

    usbTransmitQueue.frames[usbTransmitQueue.writeIndex] = *packet;
    usbTransmitQueue.writeIndex =
        (usbTransmitQueue.writeIndex + 1U) % APP_USB_TX_CAP;
    usbTransmitQueue.frameCount++;
    usbTaskDebugState.txQueuedCount++;

    if (interruptState == 0U)
    {
        __enable_irq();
    }

    (void)osThreadFlagsSet(usbTaskHandle, USB_TASK_FLAG_TRANSMIT);
}

/**
 * @brief 从发送队列取出一帧，调用者在短临界区内保证索引一致。
 */
static uint8_t USB_Task_DequeueFrame(usb_status_packet_t *packet)
{
    uint32_t interruptState = __get_PRIMASK();
    uint8_t hasFrame = 0U;

    __disable_irq();

    if (usbTransmitQueue.frameCount > 0U)
    {
        *packet = usbTransmitQueue.frames[usbTransmitQueue.readIndex];
        usbTransmitQueue.readIndex =
            (usbTransmitQueue.readIndex + 1U) % APP_USB_TX_CAP;
        usbTransmitQueue.frameCount--;
        hasFrame = 1U;
    }

    if (interruptState == 0U)
    {
        __enable_irq();
    }

    return hasFrame;
}

/**
 * @brief 将最多 16 帧状态数据连续装入一次 CDC 发送缓冲。
 */
static void USB_Task_FillTransmitBuffer(void)
{
    usb_status_packet_t packet;

    usbTransmitLength = 0U;
    usbActiveFrameCount = 0U;

    while ((usbActiveFrameCount < APP_USB_TX_BATCH) &&
           (USB_Task_DequeueFrame(&packet) != 0U))
    {
        memcpy(&usbTransmitBuffer[usbTransmitLength],
               &packet,
               sizeof(packet));
        usbTransmitLength += sizeof(packet);
        usbActiveFrameCount++;
    }
}

/**
 * @brief USB 空闲时提交一批数据；BUSY 保留本批，其他失败直接丢弃。
 */
static void USB_Task_Transmit(void)
{
    uint8_t result;

    if (usbTransmitActive != 0U)
    {
        return;
    }

    if (usbTransmitLength == 0U)
    {
        USB_Task_FillTransmitBuffer();
    }

    if (usbTransmitLength == 0U)
    {
        return;
    }

    usbTransmitActive = 1U;
    result = Driver_USB_Send(usbTransmitBuffer, usbTransmitLength);
    if (result == USBD_OK)
    {
        usbTransmitLength = 0U;
        return;
    }

    usbTransmitActive = 0U;
    if (result == USBD_BUSY)
    {
        usbTaskDebugState.txBusyCount++;
        return;
    }

    usbTaskDebugState.txErrorCount++;
    usbTaskDebugState.txDroppedCount += usbActiveFrameCount;
    usbActiveFrameCount = 0U;
    usbTransmitLength = 0U;
}

static void USB_Task_Entry(void *argument)
{
    task_imu_state_t imuState = {0};
    uint32_t statusTick = HAL_GetTick();
    uint32_t statisticsTick = HAL_GetTick();
    uint32_t previousRxByteCount = 0U;
    uint32_t previousRxFrameCount = 0U;
    uint32_t previousTxCompleteCount = 0U;

    (void)argument;

    for (;;)
    {
        uint32_t nowTick;

        (void)osThreadFlagsWait(USB_TASK_FLAG_RECEIVE | USB_TASK_FLAG_TRANSMIT,
                                osFlagsWaitAny,
                                APP_USB_WAIT_TICKS);
        USB_Task_ProcessReceivedData();

        nowTick = HAL_GetTick();
        if ((nowTick - statusTick) >= APP_USB_STATUS_TICKS)
        {
            /* USB 只通过 IMU 公开接口读取姿态数据，不依赖底盘任务内部状态。 */
            IMU_Task_GetState(&imuState);
            usbTaskTxData.roll = imuState.rollRad;
            usbTaskTxData.yaw = imuState.yawRad;
            usbTaskTxData.pitch = imuState.pitchRad;
            USB_Task_SendStatus(usbTaskTxCommandId, &usbTaskTxData);
            statusTick = nowTick;
        }

        USB_Task_Transmit();

        if ((nowTick - statisticsTick) >= 1000U)
        {
            uint32_t elapsedMs = nowTick - statisticsTick;
            uint32_t rxByteCount = usbTaskDebugState.rxByteCount;
            uint32_t rxFrameCount = usbTaskDebugState.rxFrameCount;
            uint32_t txCompleteCount = usbTaskDebugState.txCompleteCount;
            uint32_t rxFrameDelta = rxFrameCount - previousRxFrameCount;
            uint32_t txCompleteDelta = txCompleteCount - previousTxCompleteCount;
            uint32_t rxExpectedCount =
                rxFrameCount - usbTaskDebugState.rxDuplicateCount +
                usbTaskDebugState.rxLostCount;
            uint32_t txHandledCount =
                txCompleteCount + usbTaskDebugState.txDroppedCount;

            usbTaskDebugState.rxByteRateBps =
                ((rxByteCount - previousRxByteCount) * 1000U) / elapsedMs;
            usbTaskDebugState.rxFrameRateHz =
                (rxFrameDelta * 1000U) / elapsedMs;

            if (rxExpectedCount > 0U)
            {
                usbTaskDebugState.rxLossPercent =
                    ((float)usbTaskDebugState.rxLostCount * 100.0f) /
                    (float)rxExpectedCount;
            }
            else
            {
                usbTaskDebugState.rxLossPercent = 0.0f;
            }

            usbTaskDebugState.txByteRateBps =
                (txCompleteDelta * sizeof(usb_status_packet_t) * 1000U) /
                elapsedMs;
            usbTaskDebugState.txFrameRateHz =
                (txCompleteDelta * 1000U) / elapsedMs;

            if (txHandledCount > 0U)
            {
                usbTaskDebugState.txLossPercent =
                    ((float)usbTaskDebugState.txDroppedCount * 100.0f) /
                    (float)txHandledCount;
            }
            else
            {
                usbTaskDebugState.txLossPercent = 0.0f;
            }

            previousRxByteCount = rxByteCount;
            previousRxFrameCount = rxFrameCount;
            previousTxCompleteCount = txCompleteCount;
            statisticsTick = nowTick;
        }
    }
}

void USB_Task_Init(void)
{
    USB_Protocol_Init(&usbTaskDebugState.rxParser,
                      USB_Task_FrameReceived);
    Driver_USB_Init(USB_Task_ReceiveCallback,
                    USB_Task_TransmitCallback,
                    USB_Task_DisconnectCallback);
    usbTaskHandle = osThreadNew(USB_Task_Entry, NULL, &usbTaskAttributes);
}

void USB_Task_SendStatus(uint16_t commandId,
                         const usb_status_payload_t *status)
{
    usb_status_packet_t packet;

    USB_Protocol_PackStatus(commandId,
                            usbTransmitSequence,
                            status,
                            &packet);
    usbTransmitSequence++;
    USB_Task_QueueFrame(&packet);
}
