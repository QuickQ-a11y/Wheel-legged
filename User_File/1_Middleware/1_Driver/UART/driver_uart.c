#include "driver_uart.h"

#include <stddef.h>
#include <string.h>

driver_uart_object_t driverUart5Object;

/** @brief 根据 HAL 句柄取得对应的 UART 驱动对象。 */
static driver_uart_object_t *Driver_UART_GetObject(UART_HandleTypeDef *handle)
{
    if ((handle != NULL) && (handle->Instance == UART5))
    {
        return &driverUart5Object;
    }

    return NULL;
}

void Driver_UART_Init(UART_HandleTypeDef *handle,
                      uint8_t *rxBuffer,
                      uint16_t rxLength,
                      driver_uart_rx_callback_t rxCallback,
                      driver_uart_error_callback_t errorCallback)
{
    driver_uart_object_t *uartObject = Driver_UART_GetObject(handle);

    if ((uartObject == NULL) || (rxBuffer == NULL) || (rxLength == 0U))
    {
        return;
    }

    memset(uartObject, 0, sizeof(*uartObject));
    uartObject->handle = handle;
    uartObject->rxBuffer = rxBuffer;
    uartObject->rxLength = rxLength;
    uartObject->rxCallback = rxCallback;
    uartObject->errorCallback = errorCallback;
}

uint8_t Driver_UART_StartRx(UART_HandleTypeDef *handle)
{
    driver_uart_object_t *uartObject = Driver_UART_GetObject(handle);

    if ((uartObject == NULL) ||
        (uartObject->handle == NULL) ||
        (uartObject->rxBuffer == NULL) ||
        (uartObject->rxLength == 0U))
    {
        return 0U;
    }
    if (uartObject->receiving != 0U)
    {
        return 1U;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(uartObject->handle,
                                     uartObject->rxBuffer,
                                     uartObject->rxLength) != HAL_OK)
    {
        uartObject->restartErrorCount++;
        return 0U;
    }

    uartObject->receiving = 1U;
    __HAL_DMA_DISABLE_IT(uartObject->handle->hdmarx, DMA_IT_HT);
    return 1U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t size)
{
    driver_uart_object_t *uartObject = Driver_UART_GetObject(handle);

    if (uartObject == NULL)
    {
        return;
    }

    uartObject->receiving = 0U;
    uartObject->rxEventCount++;
    if (uartObject->rxCallback != NULL)
    {
        /* DMA 缓冲区会立即重新启用，上层回调必须先复制本次数据。 */
        uartObject->rxCallback(uartObject->rxBuffer, size);
    }
    (void)Driver_UART_StartRx(handle);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    driver_uart_object_t *uartObject = Driver_UART_GetObject(handle);

    if (uartObject == NULL)
    {
        return;
    }

    uartObject->receiving = 0U;
    uartObject->lastErrorCode = HAL_UART_GetError(handle);
    uartObject->errorCount++;
    (void)HAL_UART_AbortReceive(handle);

    if (uartObject->errorCallback != NULL)
    {
        uartObject->errorCallback(uartObject->lastErrorCode);
    }
    (void)Driver_UART_StartRx(handle);
}
