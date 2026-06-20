#ifndef DRIVER_SPI_H
#define DRIVER_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_status.h"
#include "spi.h"

#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *gpioPort;      /* 片选 GPIO 端口。 */
    uint16_t gpioPin;            /* 片选 GPIO 引脚。 */
} driver_spi_chip_select_t;

/**
 * @brief 在一次片选有效窗口内阻塞发送 SPI 数据。
 */
app_status_t Driver_SPI_Transmit(SPI_HandleTypeDef *handle,
                              const driver_spi_chip_select_t *chipSelect,
                              const uint8_t *data,
                              uint16_t length,
                              uint32_t timeoutMs);

/**
 * @brief 在一次片选有效窗口内阻塞同步收发 SPI 数据。
 */
app_status_t Driver_SPI_TransmitReceive(SPI_HandleTypeDef *handle,
                                     const driver_spi_chip_select_t *chipSelect,
                                     const uint8_t *txData,
                                     uint8_t *rxData,
                                     uint16_t length,
                                     uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif

#endif
