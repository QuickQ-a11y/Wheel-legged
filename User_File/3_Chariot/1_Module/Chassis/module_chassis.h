#ifndef MODULE_CHASSIS_H
#define MODULE_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

#include <stdint.h>

#define MODULE_CHASSIS_FAULT_NONE 0x00000000UL
#define MODULE_CHASSIS_FAULT_DISABLED 0x00000001UL
#define MODULE_CHASSIS_FAULT_IMU 0x00000002UL
#define MODULE_CHASSIS_FAULT_DM_MOTOR 0x00000004UL
#define MODULE_CHASSIS_FAULT_DJI_MOTOR 0x00000008UL
#define MODULE_CHASSIS_FAULT_CAN 0x00000010UL
#define MODULE_CHASSIS_FAULT_CONTROLLER 0x00000020UL

typedef struct
{
    uint8_t isInitialized;       /* IMU 已完成初始化。 */
    uint8_t isAttitudeReady;     /* 姿态估计已经完成零偏准备。 */
    uint32_t errorCode;          /* IMU 设备层最近一次错误码。 */
    float rollRad;               /* 整车右手系横滚角，单位 rad。 */
    float pitchRad;              /* 整车右手系俯仰角，单位 rad。 */
    float yawRad;                /* 整车右手系偏航角，单位 rad。 */
    float gyroRadps[APP_CONFIG_IMU_AXIS_COUNT]; /* 整车右手系去零偏角速度，单位 rad/s。 */
    float motionAccMps2[APP_CONFIG_IMU_AXIS_COUNT]; /* 整车右手系去重力运动加速度，单位 m/s^2。 */
    float dtSec;                 /* 最近一次姿态积分周期，单位 s。 */
} module_chassis_imu_state_t;

typedef struct
{
    uint8_t isOnline;            /* 电机反馈是否在线。 */
    float positionRad;           /* 关节位置，单位 rad。 */
    float velocityRadps;         /* 关节速度，单位 rad/s。 */
    float torqueNm;              /* 反馈力矩，单位 N*m。 */
} module_chassis_dm_state_t;

typedef struct
{
    uint8_t isOnline;            /* 轮电机反馈是否在线。 */
    int16_t speedRpm;            /* ESC 反馈转速，单位 rpm。 */
    int16_t currentRaw;          /* ESC 反馈电流原始值。 */
} module_chassis_dji_state_t;

typedef struct
{
    uint8_t isEnabled;           /* 外部使能开关，0 表示强制安全输出。 */
    module_chassis_imu_state_t imu;
    module_chassis_dm_state_t dmMotors[APP_CONFIG_DM_MOTOR_COUNT];
    module_chassis_dji_state_t djiWheels[APP_CONFIG_DJI_WHEEL_COUNT];
    uint32_t canTxErrorCount;    /* CAN 发送错误累计值。 */
} module_chassis_input_t;

typedef struct
{
    float torqueNm;              /* DM MIT 力矩通道命令，安全输出为 0，单位 N*m。 */
} module_chassis_dm_command_t;

typedef struct
{
    module_chassis_dm_command_t dmCommands[APP_CONFIG_DM_MOTOR_COUNT];
    int16_t djiCurrents[APP_CONFIG_DJI_WHEEL_COUNT];
    uint8_t safeOutput;          /* 1 表示输出必须保持零电流和零力矩。 */
    uint32_t faultFlags;         /* MODULE_CHASSIS_FAULT_* 位组合。 */
} module_chassis_output_t;

/**
 * @brief 初始化底盘模块状态。
 *
 * 初始化控制器状态并进入安全输出。
 */
void Module_Chassis_Init(void);

/**
 * @brief 清空底盘运动融合状态。
 *
 * 零力矩、故障或模式切出站立控制时调用，避免旧的速度和位移积分影响下次控制。
 */
void Module_Chassis_ResetMotionState(void);

/**
 * @brief 执行一轮底盘控制并生成输出。
 *
 * 包含故障检查、控制器计算和安全输出判断。第一阶段默认仍保持安全零输出。
 */
void Module_Chassis_RunControl(const module_chassis_input_t *input,
                               module_chassis_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
