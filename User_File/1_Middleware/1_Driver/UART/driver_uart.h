#ifndef DRIVER_UART_H
#define DRIVER_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#include <stdint.h>

typedef void (*driver_uart_rx_callback_t)(const uint8_t *data,
                                          uint16_t length);
typedef void (*driver_uart_error_callback_t)(uint32_t errorCode);

typedef struct
{
    UART_HandleTypeDef *handle;
    uint8_t *rxBuffer;
    uint16_t rxLength;
    driver_uart_rx_callback_t rxCallback;
    driver_uart_error_callback_t errorCallback;
    volatile uint8_t receiving;
    volatile uint32_t rxEventCount;
    volatile uint32_t errorCount;
    volatile uint32_t restartErrorCount;
    volatile uint32_t lastErrorCode;
} driver_uart_object_t;

extern driver_uart_object_t driverUart5Object;

/**
 * @brief 将接收缓冲区和回调绑定到 UART5 驱动对象。
 */
void Driver_UART_Init(UART_HandleTypeDef *handle,
                      uint8_t *rxBuffer,
                      uint16_t rxLength,
                      driver_uart_rx_callback_t rxCallback,
                      driver_uart_error_callback_t errorCallback);

/**
 * @brief 启动或恢复指定 UART 的 Receive-to-IDLE DMA 接收。
 */
uint8_t Driver_UART_StartRx(UART_HandleTypeDef *handle);

#ifdef __cplusplus
}
#endif

#endif
