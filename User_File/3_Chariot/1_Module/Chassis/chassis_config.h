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
#define CHASSIS_STATE_COUNT 10U   /* 十维整车LQR状态数量。 */
#define CHASSIS_OUTPUT_COUNT 4U   /* 左右轮力矩和左右腿摆力矩。 */

/* 左右腿数组下标，顺序必须与LQR和电机映射保持一致。 */
typedef enum
{
    CHASSIS_LEFT = 0,
    CHASSIS_RIGHT,
    CHASSIS_LEG_COUNT,
} chassis_leg_side_t;

/* 单腿两根主动杆对应的前、后髋关节下标。 */
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

/** @brief 单侧五连杆的机械尺寸。 */
typedef struct
{
    float link1_m;                    /* 前主动杆长度，单位 m。 */
    float link2_m;                    /* 前从动杆长度，单位 m。 */
    float link3_m;                    /* 后从动杆长度，单位 m。 */
    float link4_m;                    /* 后主动杆长度，单位 m。 */
    float frame_joint_distance_m;     /* 前后髋轴间距，单位 m。 */
} chassis_geometry_config_t;

/** @brief 关节电机索引以及电机量到几何量的符号映射。 */
typedef struct
{
    uint8_t motor_index;              /* 在DM反馈和命令数组中的索引。 */
    float angle_offset_rad;           /* 电机零位对应的几何角偏置，单位 rad。 */
    float angle_scale;                /* 电机位置、速度到几何正方向的比例。 */
    float torque_scale;               /* VMC几何力矩到电机正方向的比例。 */
} chassis_joint_config_t;

/** @brief 单腿机械、关节映射和正常站立目标腿长。 */
typedef struct
{
    chassis_geometry_config_t geometry;
    chassis_joint_config_t joint[CHASSIS_JOINT_COUNT];
    float target_leg_length_m;        /* 正常站立目标腿长，单位 m。 */
} chassis_leg_config_t;

/** @brief IMU任务输出到十维控制模型坐标的轴选择和符号比例。 */
typedef struct
{
    uint8_t pitch_rate_axis;          /* pitch角速度在gyro数组中的下标。 */
    uint8_t roll_rate_axis;           /* roll角速度在gyro数组中的下标。 */
    uint8_t yaw_rate_axis;            /* yaw角速度在gyro数组中的下标。 */
    float pitch_angle_scale;
    float pitch_rate_scale;
    float roll_angle_scale;
    float roll_rate_scale;
    float yaw_angle_scale;
    float yaw_rate_scale;
    uint8_t forward_accel_axis;       /* 前向运动加速度数组下标。 */
    float forward_accel_scale;        /* IMU前向加速度到整车X正方向的比例。 */
} chassis_imu_config_t;

/** @brief 轮组几何、反馈极性和轮力矩到DJI电流值的换算。 */
typedef struct
{
    float radius_m;                   /* 轮半径，单位 m。 */
    float half_track_m;               /* 半轮距，单位 m。 */
    float left_speed_scale;           /* 左轮反馈到整车前进正方向的比例。 */
    float right_speed_scale;          /* 右轮反馈到整车前进正方向的比例。 */
    float torque_limit_nm;            /* 单轮LQR力矩请求限幅，单位 N*m。 */
    float torque_to_current;          /* 轮力矩到DJI原始电流的比例。 */
    int16_t current_limit;            /* 单轮原始电流绝对值限幅。 */
} chassis_wheel_config_t;

/** @brief 前进速度和加速度二维Kalman滤波配置。 */
typedef struct
{
    uint8_t enabled;
    float initial_covariance[4];
    float process_noise[4];
    float measurement_noise[4];
    float position_speed_limit_mps;   /* 允许积分前进位移的速度上限，单位 m/s。 */
} chassis_kalman_config_t;

/** @brief 分路输出开关和最终关节力矩限幅。 */
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

/* LQR拟合使用固定调试腿长或当前五连杆实测腿长。 */
typedef enum
{
    CHASSIS_K_LENGTH_FIXED = 0,
    CHASSIS_K_LENGTH_MEASURED,
} chassis_k_length_source_t;

/** @brief 双腿长poly22增益拟合的范围、输入来源和系数表。 */
typedef struct
{
    uint8_t enabled;                 /* 0使用fixed_lqr_k，1使用poly22拟合。 */
    chassis_k_length_source_t length_source;
    float min_leg_length_m;          /* 拟合有效腿长下限，单位 m。 */
    float max_leg_length_m;          /* 拟合有效腿长上限，单位 m。 */
    float fixed_left_length_m;       /* 固定腿长调试时左侧输入，单位 m。 */
    float fixed_right_length_m;      /* 固定腿长调试时右侧输入，单位 m。 */
    float coefficients[CHASSIS_OUTPUT_COUNT][CHASSIS_STATE_COUNT]
                      [ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT];
} chassis_lqr_config_t;

/** @brief 底盘控制唯一只读配置，集中保存机械、模型和安全参数。 */
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
    float leg_vertical_offset_rad;    /* phi0换算LQR腿倾角时的竖直零点。 */
    float target_roll_rad;            /* 机体横滚目标，单位 rad。 */
    float base_support_force_n;       /* 两腿公共基础支撑力，单位 N。 */
    float left_support_feedforward_n; /* 左腿支撑力静态修正，单位 N。 */
    float right_support_feedforward_n; /* 右腿支撑力静态修正，单位 N。 */
    float default_dt_s;               /* 控制周期异常时采用的默认dt。 */
    float min_dt_s;                   /* 接受的最小控制周期，单位 s。 */
    float max_dt_s;                   /* 接受的最大控制周期，单位 s。 */
    float target_state[CHASSIS_STATE_COUNT]; /* 十维LQR默认目标状态。 */
    float fixed_lqr_k[CHASSIS_OUTPUT_COUNT][CHASSIS_STATE_COUNT]; /* 固定K。 */
} chassis_config_t;

extern const chassis_config_t chassis_config;

#ifdef __cplusplus
}
#endif

#endif
