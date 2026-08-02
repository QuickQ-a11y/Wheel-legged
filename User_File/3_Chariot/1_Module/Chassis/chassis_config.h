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
} Chassis_Side_t;

/* 单腿两根主动杆对应的前、后髋关节下标。 */
typedef enum
{
    CHASSIS_JOINT_FRONT = 0,
    CHASSIS_JOINT_BACK,
    CHASSIS_JOINT_COUNT,
} Chassis_Joint_t;

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
} Chassis_State_Index_t;

typedef enum
{
    CHASSIS_OUTPUT_LEFT_WHEEL = 0,
    CHASSIS_OUTPUT_RIGHT_WHEEL,
    CHASSIS_OUTPUT_LEFT_LEG,
    CHASSIS_OUTPUT_RIGHT_LEG,
} Chassis_Output_Index_t;

/** @brief 单侧五连杆的机械尺寸。 */
typedef struct
{
    float l1;                         /* 前主动杆长度，m。 */
    float l2;                         /* 前从动杆长度，m。 */
    float l3;                         /* 后从动杆长度，m。 */
    float l4;                         /* 后主动杆长度，m。 */
    float l5;                         /* 前后髋轴间距，m。 */
} Chassis_Geometry_Config_t;

/** @brief 关节电机索引以及电机量到几何量的符号映射。 */
typedef struct
{
    uint8_t motor_index;              /* 在DM反馈和命令数组中的索引。 */
    float angle_offset_rad;           /* 电机零位对应的几何角偏置，单位 rad。 */
    float angle_scale;                /* 电机位置、速度到几何正方向的比例。 */
    float torque_scale;               /* VMC几何力矩到电机正方向的比例。 */
} Chassis_Joint_Config_t;

/** @brief 单腿机械、关节映射和正常站立目标腿长。 */
typedef struct
{
    Chassis_Geometry_Config_t geometry;
    Chassis_Joint_Config_t joint[CHASSIS_JOINT_COUNT];
    float target_L0;                  /* 正常站立目标腿长，m。 */
} Chassis_Leg_Config_t;

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
} Chassis_IMU_Config_t;

/** @brief 轮组几何、反馈极性和轮力矩到DJI电流值的换算。 */
typedef struct
{
    float R;               /* 轮半径，m。 */
    float half_track;      /* 半轮距，m。 */
    float left_scale;      /* 左轮反馈到整车前进正方向的比例。 */
    float right_scale;     /* 右轮反馈到整车前进正方向的比例。 */
    float T_limit;         /* 单轮LQR力矩请求限幅，N*m。 */
    float T_to_I;          /* 轮力矩到DJI原始电流的比例。 */
    int16_t I_limit;       /* 单轮原始电流绝对值限幅。 */
} Chassis_Wheel_Config_t;

/** @brief 前进速度和加速度二维Kalman滤波配置。 */
typedef struct
{
    uint8_t enable_flag;
    float initial_covariance[4];
    float process_noise[4];
    float measurement_noise[4];
    float position_d_s_limit;   /* 允许积分前进位移的速度上限，单位 m/s。 */
} Chassis_Kalman_Config_t;

/** @brief 分路输出开关和最终关节力矩限幅。 */
typedef struct
{
    uint8_t joint_flag;
    uint8_t wheel_flag;
    float joint_T_limit;
} Chassis_Output_Config_t;

typedef struct
{
    float bench_L0;          /* 小板凳目标腿长，m。 */
    float extend_L0;         /* 倒地转腿阶段目标腿长，m。 */
    float bench_phi0;        /* 小板凳虚拟腿目标角，rad。 */
    float rotate_phi0;       /* 基础转腿偏移，rad。 */
    float lag_phi0;          /* 滞后腿追赶偏移，rad。 */
    float theta_diff;        /* 启用追赶的双腿theta差，rad。 */
    float theta_min;         /* 转腿完成theta下限，rad。 */
    float theta_max;         /* 转腿完成theta上限，rad。 */
    float direct_pitch;      /* 允许直接准备的pitch上限，rad。 */
    float ready_pitch;       /* 阶段完成pitch上限，rad。 */
    float phi0_min;          /* 直接准备phi0下限，rad。 */
    float phi0_max;          /* 直接准备phi0上限，rad。 */
    float L0_tol;            /* 腿长完成误差，m。 */
    float angle_tol;         /* 腿角完成误差，rad。 */
    float stable_time;       /* 条件连续保持时间，s。 */
    float fallen_timeout;    /* 倒地转腿超时，s。 */
    float prepare_timeout;   /* 小板凳准备超时，s。 */
    float L0_rate;           /* 站立目标腿长斜率，m/s。 */
    float pitch_limit;       /* 站立pitch保护阈值，rad。 */
    float stand_phi0_min;    /* 站立phi0保护下限，rad。 */
    float stand_phi0_max;    /* 站立phi0保护上限，rad。 */
    float joint_T_limit;     /* 恢复与板凳关节力矩限幅，N*m。 */
    algorithm_pid_config_t joint_angle_pid; /* 关节角度到目标速度控制器。 */
    algorithm_pid_config_t joint_speed_pid; /* 关节速度到几何力矩控制器。 */
} Chassis_Recovery_Config_t;

/** @brief 小陀螺运动边界和十维反馈缩放。 */
typedef struct
{
    float max_d_s;
    float max_d_fai;
    float scale[CHASSIS_STATE_COUNT];
} Chassis_Top_Config_t;

/** @brief 正向辅助爬台阶动作参数。 */
typedef struct
{
    float approach_L0;
    float retract_L0;
    float approach_d_s;
    float contact_T_req;
    float contact_T_fb;
    float contact_theta;
    float contact_time;
    float peak_theta;
    float recover_theta;
    float L0_tol;
    float angle_tol;
    float stable_time;
    float prepare_timeout;
    float approach_timeout;
    float climb_timeout;
    float recover_timeout;
    algorithm_pid_config_t leg_angle_pid;
} Chassis_Step_Config_t;

/** @brief 只用于Watch的打滑、离地和高速转向观测参数。 */
typedef struct
{
    float gravity_mps2;
    float body_mass_kg;
    float leg_mass_kg;
    float wheel_mass_kg;
    float body_cg_to_hip_m;
    float residual_filter_s;
    float turn_filter_s;
    float normal_force_filter_s;
    float slip_speed_enter_mps;
    float slip_speed_exit_mps;
    float slip_yaw_enter_radps;
    float slip_yaw_exit_radps;
    float slip_delta_enter_mps;
    float slip_delta_exit_mps;
    float slip_enter_s;
    float slip_exit_s;
    float off_force_ratio;
    float land_force_ratio;
    float off_hold_s;
    float land_hold_s;
    float turn_force_limit_ratio;
} Chassis_Observer_Config_t;

/* LQR拟合使用固定调试腿长或当前五连杆实测腿长。 */
typedef enum
{
    CHASSIS_K_LENGTH_FIXED = 0,
    CHASSIS_K_LENGTH_MEASURED,
} Chassis_K_Source_t;

/** @brief 双腿长poly22增益拟合的范围、输入来源和系数表。 */
typedef struct
{
    uint8_t enable_flag;                 /* 0使用fixed_K，1使用poly22拟合。 */
    Chassis_K_Source_t L0_source;
    float L0_min;                       /* 拟合腿长下限，m。 */
    float L0_max;                       /* 拟合腿长上限，m。 */
    float fixed_L0[CHASSIS_LEG_COUNT]; /* 固定调试腿长，m。 */
    float coefficients[CHASSIS_OUTPUT_COUNT][CHASSIS_STATE_COUNT]
                      [ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT];
} Chassis_LQR_Config_t;

/** @brief 底盘控制唯一只读配置，集中保存机械、模型和安全参数。 */
typedef struct
{
    Chassis_Leg_Config_t leg[CHASSIS_LEG_COUNT];
    Chassis_IMU_Config_t imu;
    Chassis_Wheel_Config_t wheel;
    Chassis_Kalman_Config_t speed_kalman;
    algorithm_pid_config_t leg_length_pid;
    algorithm_pid_config_t roll_pid;
    Chassis_Recovery_Config_t recovery;
    Chassis_Top_Config_t top;
    Chassis_Step_Config_t step;
    Chassis_Observer_Config_t observer;
    Chassis_LQR_Config_t lqr;
    Chassis_Output_Config_t output;
    float phi0_offset;                /* phi0换算theta时的竖直零点，rad。 */
    float roll_target;                /* 机体横滚目标，rad。 */
    float F0_base;                    /* 两腿公共基础支撑力，N。 */
    float F0_left;                    /* 左腿支撑力静态修正，N。 */
    float F0_right;                   /* 右腿支撑力静态修正，N。 */
    float default_dt;               /* 控制周期异常时采用的默认dt，s。 */
    float dt_min;                   /* 最小控制周期，s。 */
    float dt_max;                   /* 最大控制周期，s。 */
    float target[CHASSIS_STATE_COUNT]; /* 十维LQR默认目标状态。 */
    float fixed_K[CHASSIS_OUTPUT_COUNT][CHASSIS_STATE_COUNT]; /* 固定K。 */
} Chassis_Config_t;

extern const Chassis_Config_t Chassis_Config;

#ifdef __cplusplus
}
#endif

#endif
