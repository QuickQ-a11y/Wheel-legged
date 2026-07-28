#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Kalman.h"
#include "chassis_config.h"
#include "chassis_vmc.h"

#include <stdint.h>

#define CHASSIS_FAULT_NONE 0x00000000UL
#define CHASSIS_FAULT_DISABLED 0x00000001UL
#define CHASSIS_FAULT_IMU 0x00000002UL
#define CHASSIS_FAULT_DM_MOTOR 0x00000004UL
#define CHASSIS_FAULT_DJI_MOTOR 0x00000008UL
#define CHASSIS_FAULT_CAN 0x00000010UL
#define CHASSIS_FAULT_CONTROL 0x00000020UL

typedef enum
{
    CHASSIS_MODE_ZERO_FORCE = 0,
    CHASSIS_MODE_FOLLOW,
    CHASSIS_MODE_TOP,
    CHASSIS_MODE_SELF_SAVE,
    CHASSIS_MODE_BENCH,
} chassis_mode_t;

typedef enum
{
    CHASSIS_STANDING = 0,
    CHASSIS_ZERO_FORCE,
    CHASSIS_FALLEN,
    CHASSIS_FALLING_TO_STAND,
    CHASSIS_BENCH,
} chassis_control_state_t;

typedef struct
{
    uint8_t initialized;
    uint8_t attitude_ready;
    uint32_t error_code;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float gyro_radps[APP_IMU_AXIS_COUNT];
    float motion_accel_mps2[APP_IMU_AXIS_COUNT];
} chassis_imu_t;

typedef struct
{
    uint8_t online;
    float position_rad;
    float speed_radps;
    float torque_nm;
} chassis_dm_motor_t;

typedef struct
{
    uint8_t online;
    int16_t speed_rpm;
    int16_t current;
} chassis_dji_motor_t;

typedef struct
{
    uint8_t enabled;
    chassis_mode_t mode;
    chassis_control_state_t state;

    chassis_imu_t imu;
    chassis_dm_motor_t dm_motor[APP_DM_COUNT];
    chassis_dji_motor_t wheel_motor[APP_WHEEL_COUNT];
    uint32_t can_tx_error_count;
    float control_dt_s;            /* 底盘控制本轮实际周期，单位 s。 */

    chassis_vmc_state_t leg[CHASSIS_LEG_COUNT];
    float lqr_state[CHASSIS_STATE_COUNT];
    float lqr_k[CHASSIS_OUTPUT_COUNT][CHASSIS_STATE_COUNT];
    float lqr_output[CHASSIS_OUTPUT_COUNT];
    float support_force_n[CHASSIS_LEG_COUNT];
    float k_input_length_m[CHASSIS_LEG_COUNT];
    float k_limited_length_m[CHASSIS_LEG_COUNT];
    float wheel_speed_radps[APP_WHEEL_COUNT];
    float forward_speed_raw_mps;
    float forward_speed_mps;
    float forward_accel_mps2;
    float forward_accel_fused_mps2;
    float forward_position_m;

    float joint_torque_nm[APP_DM_COUNT];
    int16_t wheel_current[APP_WHEEL_COUNT];
    uint8_t safe_output;
    uint32_t fault_flags;
    uint8_t k_fit_enabled;
    uint8_t k_length_limited;
    uint8_t state_valid;

    algorithm_kalman_t speed_kalman;
    algorithm_pid_state_t leg_length_pid[CHASSIS_LEG_COUNT];
    algorithm_pid_state_t roll_pid;
} chassis_t;

/* 底盘唯一实际状态，也是 Watch 窗口的长期调试入口。 */
extern chassis_t chassis;

/**
 * @brief 初始化底盘控制状态、PID 和速度卡尔曼滤波器。
 */
void chassis_control_init(void);

/**
 * @brief 清空速度融合和前进位移状态。
 */
void chassis_control_reset(void);

/**
 * @brief 执行一轮十维 LQR、支撑力和 VMC 控制计算。
 */
void chassis_control_loop(void);

/**
 * @brief 将底盘电机命令清零并重置运动融合状态。
 */
void chassis_zero_output(void);

#ifdef __cplusplus
}
#endif

#endif
