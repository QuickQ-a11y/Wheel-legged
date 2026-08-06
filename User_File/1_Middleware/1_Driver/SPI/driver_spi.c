#include "driver_spi.h"

void Driver_SPI_Transmit(SPI_HandleTypeDef *handle,
                         const driver_spi_chip_select_t *chipSelect,
                         const uint8_t *data,
                         uint16_t length,
                         uint32_t timeoutMs)
{
    HAL_GPIO_WritePin(chipSelect->gpioPort, chipSelect->gpioPin, GPIO_PIN_RESET);
    (void)HAL_SPI_Transmit(handle, (uint8_t *)data, length, timeoutMs);
    HAL_GPIO_WritePin(chipSelect->gpioPort, chipSelect->gpioPin, GPIO_PIN_SET);
}

void Driver_SPI_TransmitReceive(SPI_HandleTypeDef *handle,
                                const driver_spi_chip_select_t *chipSelect,
                                const uint8_t *txData,
                                uint8_t *rxData,
                                uint16_t length,
                                uint32_t timeoutMs)
{
    HAL_GPIO_WritePin(chipSelect->gpioPort, chipSelect->gpioPin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(handle, (uint8_t *)txData, rxData, length, timeoutMs);
    HAL_GPIO_WritePin(chipSelect->gpioPort, chipSelect->gpioPin, GPIO_PIN_SET);
}
