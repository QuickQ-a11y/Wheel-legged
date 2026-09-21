#ifndef DEVICE_MOTOR_DM_H
#define DEVICE_MOTOR_DM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

#include <stdint.h>

typedef enum
{
    MOTOR_DM_YAW = 0,
    MOTOR_DM_PITCH,
    MOTOR_DM_COUNT,
} motor_dm_index_t;

typedef struct
{
    app_can_bus_t bus;           /* 电机所在 CAN 总线。 */
    uint32_t commandId;          /* MIT控制帧标准ID。 */
    uint32_t feedbackId;         /* 调试助手配置的反馈帧标准ID。 */
    /*
     * MIT 位置字段的映射范围，单位 rad，必须与调试助手里这台电机的 PMAX 逐位一致。
     * 每台电机单独声明而不是共用一个全局宏：需求会分化——要连续转的轴必须取 pi
     * 才能让 Algorithm_AngleUnwrapRad 的 ±pi 定义域假设成立，行程小的轴没这个约束。
     *
     * 新增电机时这两项必须显式填写。漏填会静默变成 0，位置随即恒为 0，而且
     * 编译器不会报：固件只开 -Wall（没有 -Wextra），指定初始化器即使开了
     * -Wmissing-field-initializers 也不告警，本文件又无法在主机单测里编译。
     */
    float positionMin;
    float positionMax;
} motor_dm_config_t;

typedef struct
{
    float positionWrappedRad;    /* 反馈帧解码位置，范围 [config.positionMin, positionMax] rad；
                                  * 到端点会直接跳到另一端，连续角看 positionRad。 */
    float positionRad;           /* 本次在线期间连续展开的位置，单位 rad。 */
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
    float torqueNm;              /* MIT 力矩通道命令，单位 N*m。 */
} motor_dm_command_t;

/**
 * @brief 初始化云台 Yaw、Pitch 两台 DM 电机的配置和状态。
 */
void Motor_DM_Init(void);

/**
 * @brief 尝试解析一帧 DM MIT 反馈，成功匹配并更新状态时返回 1。
 */
uint8_t Motor_DM_UpdateFeedback(app_can_bus_t bus,
                                uint32_t identifier,
                                const uint8_t data[APP_DM_FRAME_LEN]);

/**
 * @brief 设置单个 DM 电机命令缓存。
 */
void Motor_DM_SetCommand(motor_dm_index_t index,
                         const motor_dm_command_t *command);

/**
 * @brief 读取单个 DM 电机状态快照。
 */
void Motor_DM_GetState(motor_dm_index_t index,
                       motor_dm_state_t *state);

void Motor_DM_SetSafe(uint8_t safe);
void Motor_DM_SetEnable(uint8_t enable);
uint8_t Motor_DM_IsOnline(motor_dm_index_t index, uint32_t nowTick);

/** @brief 清空软件保存的两路力矩命令，不修改电机零点。 */
void Motor_DM_ClearCommands(void);

/**
 * @brief 将当前 DM 命令打包并更新到 CAN 发送缓存。
 */
void Motor_DM_UpdateTxFrames(void);

#ifdef __cplusplus
}
#endif

#endif
