#ifndef DEVICE_MOTOR_DJI_H
#define DEVICE_MOTOR_DJI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

#include <stdint.h>

typedef struct
{
    uint16_t encoderRaw;         /* ESC 编码器机械角度，范围 0..8191。 */
    uint16_t lastEncoderRaw;     /* 上一次编码器值，用于后续扩展回绕处理。 */
    int16_t speedRpm;            /* ESC 反馈转速，单位 rpm。 */
    int16_t currentRaw;          /* ESC 反馈电流原始值。 */
    uint8_t temperatureCelsius;  /* 电机温度，单位摄氏度。 */
    uint32_t feedbackCount;      /* 有效反馈帧累计数量。 */
    uint32_t lastUpdateTick;     /* 最近一次反馈 HAL tick。 */
    uint8_t isOnline;            /* 收到有效反馈后置 1。 */
} motor_dji_t;

/**
 * @brief 解析一帧 DJI ESC 反馈并刷新电机状态。
 *
 * 反馈帧为大端格式：编码器、转速、电流各 16 位，温度 8 位。
 */
void Motor_DJI_UpdateFeedback(motor_dji_t *motor,
                              const uint8_t data[APP_CONFIG_DJI_FEEDBACK_LENGTH]);

/**
 * @brief 判断 DJI 电机反馈是否仍在线。
 */
uint8_t Motor_DJI_IsOnline(const motor_dji_t *motor, uint32_t nowTick);

#ifdef __cplusplus
}
#endif

#endif
