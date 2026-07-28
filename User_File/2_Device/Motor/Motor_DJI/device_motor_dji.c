#include "device_motor_dji.h"

#include "stm32h7xx_hal.h"

/**
 * @brief 从两个大端字节解析有符号 16 位数据。
 */
static int16_t Motor_DJI_MakeInt16(uint8_t highByte, uint8_t lowByte)
{
    return (int16_t)(((uint16_t)highByte << 8U) | (uint16_t)lowByte);
}

void Motor_DJI_UpdateFeedback(motor_dji_t *motor,
                              const uint8_t data[APP_DJI_RX_LEN])
{
    if ((motor == NULL) || (data == NULL))
    {
        return;
    }

    motor->lastEncoderRaw = motor->encoderRaw;
    motor->encoderRaw = (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
    motor->speedRpm = Motor_DJI_MakeInt16(data[2], data[3]);
    motor->currentRaw = Motor_DJI_MakeInt16(data[4], data[5]);
    motor->temperatureCelsius = data[6];
    motor->feedbackCount++;
    motor->lastUpdateTick = HAL_GetTick();
    motor->isOnline = 1U;
}

uint8_t Motor_DJI_IsOnline(const motor_dji_t *motor, uint32_t nowTick)
{
    if ((motor == NULL) || (motor->isOnline == 0U))
    {
        return 0U;
    }

    return ((nowTick - motor->lastUpdateTick) <= APP_DJI_TIMEOUT_TICKS) ? 1U : 0U;
}
