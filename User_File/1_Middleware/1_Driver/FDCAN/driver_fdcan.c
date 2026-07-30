#include "driver_fdcan.h"

#include <string.h>

driver_fdcan_object_t driverFdcan1Object = {0};
driver_fdcan_object_t driverFdcan2Object = {0};

/**
 * @brief 根据 HAL 句柄找到对应管理对象。
 *
 * 当前硬件只使用 FDCAN1 和 FDCAN2。
 */
static driver_fdcan_object_t *Driver_FDCAN_GetObject(FDCAN_HandleTypeDef *handle)
{
    if (handle == NULL)
    {
        return NULL;
    }

    if (handle->Instance == FDCAN1)
    {
        return &driverFdcan1Object;
    }

    if (handle->Instance == FDCAN2)
    {
        return &driverFdcan2Object;
    }

    return NULL;
}

/**
 * @brief 将 0..8 字节长度转换为 HAL DLC 常量。
 */
static uint32_t Driver_FDCAN_LengthToDlc(uint8_t length)
{
    static const uint32_t dlcTable[APP_CAN_DATA_MAX_BYTES + 1U] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8,
    };

    return dlcTable[length];
}

/**
 * @brief 配置接收所有标准数据帧到 RX FIFO0。
 */
static void Driver_FDCAN_ConfigStdFilter(FDCAN_HandleTypeDef *handle)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    /* 掩码为 0 表示不比较任何 ID 位，由应用层按总线和协议分发。 */
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x000U;

    (void)HAL_FDCAN_ConfigFilter(handle, &filter);
    (void)HAL_FDCAN_ConfigGlobalFilter(handle,
                                       FDCAN_REJECT,
                                       FDCAN_REJECT,
                                       FDCAN_REJECT_REMOTE,
                                       FDCAN_REJECT_REMOTE);
}

/**
 * @brief 读取指定 FIFO 中所有待处理帧并交给上层。
 */
static void Driver_FDCAN_DispatchRxFifo(FDCAN_HandleTypeDef *handle,
                                        uint32_t rxFifo)
{
    driver_fdcan_object_t *fdcanObject = Driver_FDCAN_GetObject(handle);

    if (fdcanObject == NULL)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(handle, rxFifo) > 0U)
    {
        memset(fdcanObject->rxFrame.data, 0, sizeof(fdcanObject->rxFrame.data));

        if (HAL_FDCAN_GetRxMessage(handle,
                                   rxFifo,
                                   &fdcanObject->rxFrame.header,
                                   fdcanObject->rxFrame.data) != HAL_OK)
        {
            return;
        }

        if (fdcanObject->rxCallback != NULL)
        {
            fdcanObject->rxCallback(handle, &fdcanObject->rxFrame);
        }
    }
}

void Driver_FDCAN_Init(FDCAN_HandleTypeDef *handle,
                       driver_fdcan_rx_callback_t rxCallback)
{
    driver_fdcan_object_t *fdcanObject = Driver_FDCAN_GetObject(handle);

    if (fdcanObject == NULL)
    {
        return;
    }

    fdcanObject->handle = handle;
    fdcanObject->rxCallback = rxCallback;

    Driver_FDCAN_ConfigStdFilter(handle);
    (void)HAL_FDCAN_ConfigInterruptLines(handle,
                                         FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                         FDCAN_INTERRUPT_LINE0);
    (void)HAL_FDCAN_Start(handle);
    (void)HAL_FDCAN_ActivateNotification(handle,
                                         FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                         0U);
}

void Driver_FDCAN_RegisterRxCallback(FDCAN_HandleTypeDef *handle,
                                     driver_fdcan_rx_callback_t rxCallback)
{
    driver_fdcan_object_t *fdcanObject = Driver_FDCAN_GetObject(handle);

    if (fdcanObject == NULL)
    {
        return;
    }

    fdcanObject->handle = handle;
    fdcanObject->rxCallback = rxCallback;
}

void Driver_FDCAN_SendData(FDCAN_HandleTypeDef *handle,
                           uint32_t identifier,
                           const uint8_t *data,
                           uint8_t length)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    FDCAN_ProtocolStatusTypeDef protocolStatus = {0};
    uint8_t emptyData[APP_CAN_DATA_MAX_BYTES] = {0};
    const uint8_t *txData = data;
    uint32_t pendingTxRequests;

    if ((handle == NULL) ||
        (identifier > APP_CAN_STD_ID_MAX) ||
        (length > APP_CAN_DATA_MAX_BYTES) ||
        ((data == NULL) && (length > 0U)))
    {
        return;
    }

    (void)HAL_FDCAN_GetProtocolStatus(handle, &protocolStatus);
    if (protocolStatus.BusOff != 0U)
    {
        /*
         * 无 ACK 或总线错误可能让控制器进入 BusOff。
         * 当前帧直接丢弃，只恢复外设，下一周期继续发送最新命令。
         */
        (void)HAL_FDCAN_Stop(handle);
        (void)HAL_FDCAN_Start(handle);
        (void)HAL_FDCAN_ActivateNotification(handle,
                                             FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                             0U);
        return;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0U)
    {
        /*
         * 旧发送请求可能因为无 ACK 或错误帧占住 Tx FIFO。
         * 取消 pending 请求后丢弃当前帧，避免旧帧阻塞后续控制命令。
         */
        pendingTxRequests = handle->Instance->TXBRP;
        if (pendingTxRequests != 0U)
        {
            (void)HAL_FDCAN_AbortTxRequest(handle, pendingTxRequests);
        }
        return;
    }

    if (txData == NULL)
    {
        txData = emptyData;
    }

    txHeader.Identifier = identifier;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = Driver_FDCAN_LengthToDlc(length);
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &txHeader, (uint8_t *)txData) != HAL_OK)
    {
        pendingTxRequests = handle->Instance->TXBRP;
        if (pendingTxRequests != 0U)
        {
            (void)HAL_FDCAN_AbortTxRequest(handle, pendingTxRequests);
        }
    }
}

uint8_t Driver_FDCAN_DlcToLength(uint32_t dataLength)
{
    switch (dataLength)
    {
    case FDCAN_DLC_BYTES_0:
        return 0U;
    case FDCAN_DLC_BYTES_1:
        return 1U;
    case FDCAN_DLC_BYTES_2:
        return 2U;
    case FDCAN_DLC_BYTES_3:
        return 3U;
    case FDCAN_DLC_BYTES_4:
        return 4U;
    case FDCAN_DLC_BYTES_5:
        return 5U;
    case FDCAN_DLC_BYTES_6:
        return 6U;
    case FDCAN_DLC_BYTES_7:
        return 7U;
    case FDCAN_DLC_BYTES_8:
        return 8U;
    default:
        return 0U;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t rxFifo0ItFlags)
{
    if ((rxFifo0ItFlags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
    {
        Driver_FDCAN_DispatchRxFifo(handle, FDCAN_RX_FIFO0);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *handle, uint32_t rxFifo1ItFlags)
{
    if ((rxFifo1ItFlags & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U)
    {
        Driver_FDCAN_DispatchRxFifo(handle, FDCAN_RX_FIFO1);
    }
}
