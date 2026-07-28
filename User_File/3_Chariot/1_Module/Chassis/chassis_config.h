#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "LQR.h"
#include "PID.h"

#include <stdint.h>

#define CHASSIS_PI 3.14159265358979323846f
#define CHASSIS_HALF_PI 1.57079632679489661923f
#define CHASSIS_STATE_COUNT 10U
#define CHASSIS_OUTPUT_COUNT 4U

typedef enum
{
    CHASSIS_LEFT = 0,
    CHASSIS_RIGHT,
    CHASSIS_LEG_COUNT,
} chassis_leg_side_t;

typedef enum
{
    CHASSIS_JOINT_FRONT = 0,
    CHASSIS_JOINT_BACK,
    CHASSIS_JOINT_COUNT,
} chassis_joint_t;

typedef enum
{
    CHASSIS_STATE_S = 0,          /* 整车水平位移 s，单位 m。 */
    CHASSIS_STATE_DOT_S,          /* 整车水平速度 dot_s，单位 m/s。 */
    CHASSIS_STATE_FAI,            /* 整车偏航角 fai，单位 rad。 */
    CHASSIS_STATE_DOT_FAI,        /* 整车偏航角速度 dot_fai，单位 rad/s。 */
    CHASSIS_STATE_THETA_L,        /* 左虚拟腿倾角 theta_l，单位 rad。 */
    CHASSIS_STATE_DOT_THETA_L,    /* 左虚拟腿倾角速度 dot_theta_l，单位 rad/s。 */
    CHASSIS_STATE_THETA_R,        /* 右虚拟腿倾角 theta_r，单位 rad。 */
    CHASSIS_STATE_DOT_THETA_R,    /* 右虚拟腿倾角速度 dot_theta_r，单位 rad/s。 */
    CHASSIS_STATE_THETA_B,        /* 机体俯仰角 theta_b，单位 rad。 */
    CHASSIS_STATE_DOT_THETA_B,    /* 机体俯仰角速度 dot_theta_b，单位 rad/s。 */
} chassis_state_index_t;

typedef enum
{
    CHASSIS_OUTPUT_LEFT_WHEEL = 0,
    CHASSIS_OUTPUT_RIGHT_WHEEL,
    CHASSIS_OUTPUT_LEFT_LEG,
    CHASSIS_OUTPUT_RIGHT_LEG,
} chassis_output_index_t;

typedef struct
{
    float link1_m;
    float link2_m;
    float link3_m;
    float link4_m;
    float frame_joint_distance_m;
    float min_leg_length_m;
} chassis_geometry_config_t;

typedef struct
{
    uint8_t motor_index;
    float angle_offset_rad;
    float angle_scale;
    float torque_scale;
} chassis_joint_config_t;

typedef struct
{
    chassis_geometry_config_t geometry;
    chassis_joint_config_t joint[CHASSIS_JOINT_COUNT];
    float target_leg_length_m;
} chassis_leg_config_t;

typedef struct
{
    uint8_t pitch_rate_axis;
    uint8_t roll_rate_axis;
    uint8_t yaw_rate_axis;
    float pitch_angle_scale;
    float pitch_rate_scale;
    float roll_angle_scale;
    float roll_rate_scale;
    float yaw_angle_scale;
    float yaw_rate_scale;
    uint8_t forward_accel_axis;
    float forward_accel_scale;
} chassis_imu_config_t;

typedef struct
{
    float radius_m;
    float half_track_m;
    float left_speed_scale;
    float right_speed_scale;
    float torque_limit_nm;
    float torque_to_current;
    int16_t current_limit;
} chassis_wheel_config_t;

typedef struct
{
    uint8_t enabled;
    float initial_covariance[4];
    float process_noise[4];
    float measurement_noise[4];
    float position_speed_limit_mps;
} chassis_kalman_config_t;

typedef struct
{
    uint8_t joint_enabled;
    uint8_t wheel_enabled;
    float joint_torque_limit_nm;
} chassis_motor_output_config_t;

typedef struct
{
    float bench_leg_length_m;             /* 小板凳目标腿长，单位 m。 */
    float extended_leg_length_m;          /* 倒地转腿阶段目标腿长，单位 m。 */
    float bench_phi0_rad;                  /* 小板凳虚拟腿目标角，单位 rad。 */
    float rotate_offset_rad;               /* 基础转腿目标相对当前腿角的偏移，单位 rad。 */
    float lagging_rotate_offset_rad;       /* 左右差异过大时滞后腿的转腿偏移，单位 rad。 */
    float leg_difference_threshold_rad;   /* 启用滞后腿加速的 theta 差阈值，单位 rad。 */
    float ready_theta_min_rad;             /* 转腿完成时 theta 下限，单位 rad。 */
    float ready_theta_max_rad;             /* 转腿完成时 theta 上限，单位 rad。 */
    float direct_prepare_pitch_rad;        /* 允许直接进入小板凳准备的 pitch 上限，单位 rad。 */
    float ready_pitch_rad;                 /* 阶段完成时 pitch 上限，单位 rad。 */
    float direct_phi0_min_rad;             /* 允许直接进入准备阶段的 phi0 下限，单位 rad。 */
    float direct_phi0_max_rad;             /* 允许直接进入准备阶段的 phi0 上限，单位 rad。 */
    float leg_length_tolerance_m;          /* 小板凳腿长完成误差，单位 m。 */
    float leg_angle_tolerance_rad;         /* 小板凳腿角完成误差，单位 rad。 */
    float stable_time_s;                   /* 完成条件必须连续保持的时间，单位 s。 */
    float fallen_timeout_s;                /* 倒地转腿阶段超时，单位 s。 */
    float prepare_timeout_s;               /* 小板凳准备阶段超时，单位 s。 */
    float standing_length_rate_mps;        /* 进入站立后腿长目标恢复斜率，单位 m/s。 */
    float standing_pitch_limit_rad;        /* 站立状态 pitch 保护阈值，单位 rad。 */
    float standing_phi0_min_rad;           /* 站立状态 phi0 保护下限，单位 rad。 */
    float standing_phi0_max_rad;           /* 站立状态 phi0 保护上限，单位 rad。 */
    float joint_torque_limit_nm;           /* 恢复与板凳模式关节力矩请求限幅，单位 N*m。 */
    algorithm_pid_config_t joint_angle_pid; /* 关节角度到目标速度控制器。 */
    algorithm_pid_config_t joint_speed_pid; /* 关节速度到几何力矩控制器。 */
} chassis_recovery_config_t;

typedef enum
{
    CHASSIS_K_LENGTH_FIXED = 0,
    CHASSIS_K_LENGTH_MEASURED,
} chassis_k_length_source_t;

typedef struct
{
    uint8_t enabled;
    chassis_k_length_source_t length_source;
    float min_leg_length_m;
    float max_leg_length_m;
    float fixed_left_length_m;
    float fixed_right_length_m;
    float coefficients[CHASSIS_OUTPUT_COUNT][CHASSIS_STATE_COUNT]
                      [ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT];
} chassis_lqr_config_t;

typedef struct
{
    chassis_leg_config_t leg[CHASSIS_LEG_COUNT];
    chassis_imu_config_t imu;
    chassis_wheel_config_t wheel;
    chassis_kalman_config_t speed_kalman;
    algorithm_pid_config_t leg_length_pid;
    algorithm_pid_config_t roll_pid;
    chassis_recovery_config_t recovery;
    chassis_lqr_config_t lqr;
    chassis_motor_output_config_t output;
    float leg_vertical_offset_rad;
    float target_roll_rad;
    float base_support_force_n;
    float left_support_feedforward_n;
    float right_support_feedforward_n;
    float default_dt_s;
    float min_dt_s;
    float max_dt_s;
    float target_state[CHASSIS_STATE_COUNT];
    float fixed_lqr_k[CHASSIS_OUTPUT_COUNT][CHASSIS_STATE_COUNT];
} chassis_config_t;

extern const chassis_config_t chassis_config;

#ifdef __cplusplus
}
#endif

#endif
