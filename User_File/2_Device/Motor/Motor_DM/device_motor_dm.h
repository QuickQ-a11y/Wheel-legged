#ifndef DEVICE_MOTOR_DM_H
#define DEVICE_MOTOR_DM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "app_status.h"

#include <stdint.h>

typedef enum
{
    MOTOR_DM_LEFT_FRONT = 0,
    MOTOR_DM_LEFT_BACK,
    MOTOR_DM_RIGHT_FRONT,
    MOTOR_DM_RIGHT_BACK,
    MOTOR_DM_COUNT,
} motor_dm_index_t;

typedef struct
{
    app_can_bus_t bus;           /* 电机所在 CAN 总线。 */
    uint32_t txId;               /* MIT 控制帧发送标准 ID。 */
    uint32_t rxId;               /* MIT 反馈帧接收标准 ID。 */
    int8_t direction;            /* 机械安装方向，1 或 -1。 */
} motor_dm_config_t;

typedef struct
{
    float positionRad;           /* 反馈位置，单位 rad。 */
    float velocityRadps;         /* 反馈速度，单位 rad/s。 */
    float torqueNm;              /* 反馈力矩，单位 N*m。 */
    uint8_t state;               /* DM 反馈状态高 4 位。 */
    uint8_t mosTemperature;      /* MOS 温度原始字段。 */
    uint8_t rotorTemperature;    /* 转子温度原始字段。 */
    uint32_t feedbackCount;      /* 有效反馈帧累计数量。 */
    uint32_t lastUpdateTick;     /* 最近一次反馈 HAL tick。 */
    uint8_t isOnline;            /* 收到有效反馈后置 1。 */
} motor_dm_state_t;

typedef struct
{
    float positionRad;           /* MIT 位置目标，单位 rad。 */
    float velocityRadps;         /* MIT 速度目标，单位 rad/s。 */
    float kp;                    /* MIT 位置增益。 */
    float kd;                    /* MIT 速度增益。 */
    float torqueNm;              /* MIT 前馈力矩，单位 N*m。 */
} motor_dm_command_t;

/**
 * @brief 初始化 4 个 DM 髋关节电机配置和状态。
 */
void Motor_DM_Init(void);

/**
 * @brief 解析一帧 DM MIT 反馈。
 */
app_status_t Motor_DM_UpdateFeedback(app_can_bus_t bus,
                                     uint32_t rxId,
                                     const uint8_t data[APP_CONFIG_DM_FRAME_LENGTH]);

/**
 * @brief 设置单个 DM 电机命令缓存。
 */
app_status_t Motor_DM_SetCommand(motor_dm_index_t index,
                                 const motor_dm_command_t *command);

/**
 * @brief 读取单个 DM 电机状态快照。
 */
app_status_t Motor_DM_GetState(motor_dm_index_t index,
                               motor_dm_state_t *state);

void Motor_DM_SetSafe(uint8_t safe);
uint8_t Motor_DM_IsSafe(void);
void Motor_DM_SetEnable(uint8_t enable);
uint8_t Motor_DM_IsEnabled(void);
uint8_t Motor_DM_IsOnline(motor_dm_index_t index, uint32_t nowTick);
void Motor_DM_ZeroAll(void);

/**
 * @brief 将当前 DM 命令打包并更新到 CAN 发送缓存。
 */
void Motor_DM_UpdateTxFrames(void);

#ifdef __cplusplus
}
#endif

#endif
