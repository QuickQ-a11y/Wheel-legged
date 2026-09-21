#include "device_motor_dji.h"

#include "stm32h7xx_hal.h"

/* 两路轮电机状态，反馈由 CAN 分发写入，上层只通过接口读取。 */
static motor_dji_state_t djiWheels[APP_WHEEL_COUNT];

void Motor_DJI_UpdateFeedback(motor_dji_index_t index,
                              const uint8_t data[APP_DJI_RX_LEN])
{
    motor_dji_state_t *motor = &djiWheels[index];

    motor->lastEncoderRaw = motor->encoderRaw;
    motor->encoderRaw = (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
    motor->speedRpm = (int16_t)(((uint16_t)data[2] << 8U) | (uint16_t)data[3]);
    motor->currentRaw = (int16_t)(((uint16_t)data[4] << 8U) | (uint16_t)data[5]);
    motor->temperatureCelsius = data[6];
    motor->feedbackCount++;
    motor->lastUpdateTick = HAL_GetTick();
    motor->isOnline = 1U;
}

void Motor_DJI_GetState(motor_dji_index_t index, motor_dji_state_t *state)
{
    *state = djiWheels[index];
}

uint8_t Motor_DJI_IsOnline(motor_dji_index_t index, uint32_t nowTick)
{
    if (djiWheels[index].isOnline == 0U)
    {
        return 0U;
    }

    return ((nowTick - djiWheels[index].lastUpdateTick) <=
            APP_DJI_TIMEOUT_TICKS) ? 1U : 0U;
}
