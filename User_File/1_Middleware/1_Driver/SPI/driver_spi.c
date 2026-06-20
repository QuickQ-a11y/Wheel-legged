#include "driver_spi.h"

/**
 * @brief 将 HAL SPI 返回值转换为应用层统一状态。
 */
static app_status_t Driver_SPI_ConvertHalStatus(HAL_StatusTypeDef halStatus)
{
    if (halStatus == HAL_OK)
    {
        return APP_STATUS_OK;
    }

    if (halStatus == HAL_BUSY)
    {
        return APP_STATUS_BUSY;
    }

    if (halStatus == HAL_TIMEOUT)
    {
        return APP_STATUS_TIMEOUT;
    }

    return APP_STATUS_ERROR;
}

/**
 * @brief 检查片选配置是否有效。
 */
static uint8_t Driver_SPI_IsValidChipSelect(const driver_spi_chip_select_t *chipSelect)
{
    return ((chipSelect != NULL) && (chipSelect->gpioPort != NULL)) ? 1U : 0U;
}

app_status_t Driver_SPI_Transmit(SPI_HandleTypeDef *handle,
                              const driver_spi_chip_select_t *chipSelect,
                              const uint8_t *data,
                              uint16_t length,
                              uint32_t timeoutMs)
{
    HAL_StatusTypeDef halStatus;

    if ((handle == NULL) ||
        (Driver_SPI_IsValidChipSelect(chipSelect) == 0U) ||
        (data == NULL) ||
        (length == 0U))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    HAL_GPIO_WritePin(chipSelect->gpioPort, chipSelect->gpioPin, GPIO_PIN_RESET);
    halStatus = HAL_SPI_Transmit(handle, (uint8_t *)data, length, timeoutMs);
    HAL_GPIO_WritePin(chipSelect->gpioPort, chipSelect->gpioPin, GPIO_PIN_SET);

    return Driver_SPI_ConvertHalStatus(halStatus);
}

app_status_t Driver_SPI_TransmitReceive(SPI_HandleTypeDef *handle,
                                     const driver_spi_chip_select_t *chipSelect,
                                     const uint8_t *txData,
                                     uint8_t *rxData,
                                     uint16_t length,
                                     uint32_t timeoutMs)
{
    HAL_StatusTypeDef halStatus;

    if ((handle == NULL) ||
        (Driver_SPI_IsValidChipSelect(chipSelect) == 0U) ||
        (txData == NULL) ||
        (rxData == NULL) ||
        (length == 0U))
    {
        return APP_STATUS_INVALID_PARAM;
    }

    HAL_GPIO_WritePin(chipSelect->gpioPort, chipSelect->gpioPin, GPIO_PIN_RESET);
    halStatus = HAL_SPI_TransmitReceive(handle, (uint8_t *)txData, rxData, length, timeoutMs);
    HAL_GPIO_WritePin(chipSelect->gpioPort, chipSelect->gpioPin, GPIO_PIN_SET);

    return Driver_SPI_ConvertHalStatus(halStatus);
}
