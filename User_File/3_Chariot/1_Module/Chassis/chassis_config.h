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

/* 单腿两根主动杆按五连杆共同符号定义的关节下标。 */
typedef enum
{
    CHASSIS_JOINT_PHI1 = 0,
    CHASSIS_JOINT_PHI4,
    CHASSIS_JOINT_COUNT,
} Chassis_Joint_t;

typedef enum
{
    CHASSIS_STATE_S = 0,       /* 整车水平位移 s，单位 m。 */
    CHASSIS_STATE_D_S,         /* 整车水平速度 d_s，单位 m/s。 */
    CHASSIS_STATE_FAI,         /* 整车偏航角 fai，单位 rad。 */
    CHASSIS_STATE_D_FAI,       /* 整车偏航角速度 d_fai，单位 rad/s。 */
    CHASSIS_STATE_THETA_L,     /* 左虚拟腿倾角 theta_l，单位 rad。 */
    CHASSIS_STATE_D_THETA_L,   /* 左虚拟腿倾角速度 d_theta_l，单位 rad/s。 */
    CHASSIS_STATE_THETA_R,     /* 右虚拟腿倾角 theta_r，单位 rad。 */
    CHASSIS_STATE_D_THETA_R,   /* 右虚拟腿倾角速度 d_theta_r，单位 rad/s。 */
    CHASSIS_STATE_THETA_B,     /* 机体俯仰角 theta_b，单位 rad。 */
    CHASSIS_STATE_D_THETA_B,   /* 机体俯仰角速度 d_theta_b，单位 rad/s。 */
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
    float l1;                         /* phi1主动杆长度，m。 */
    float l2;                         /* phi1侧从动杆长度，m。 */
    float l3;                         /* phi4侧从动杆长度，m。 */
    float l4;                         /* phi4主动杆长度，m。 */
    float l5;                         /* 两个主动杆输出铰点间距，m。 */
} Chassis_Geometry_Config_t;

/** @brief 关节电机索引以及电机量到几何量的符号映射。 */
typedef struct
{
    uint8_t motor_index;              /* 在DM反馈和命令数组中的索引。 */
    float angle_offset_rad;           /* 电机零位对应的几何角偏置，单位 rad。 */
    float scale;                      /* 电机与几何关节的方向，取+1或-1。 */
    float ratio;                      /* 几何关节角/电机角传动比。 */
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
    uint8_t lateral_accel_axis;       /* 横向运动加速度数组下标，供转向观测。 */
    uint8_t vertical_accel_axis;      /* 竖直运动加速度数组下标，供支撑力观测。 */
} Chassis_IMU_Config_t;

/** @brief 轮组几何、反馈极性和轮力矩到DJI电流值的换算。 */
typedef struct
{
    float R;               /* 轮半径，m。 */
    float half_track;      /* 半轮距，m。 */
    float gear_ratio;      /* 电机转子转速/轮轴转速传动比，直驱为1。 */
    float left_scale;      /* 左轮反馈和命令到模型正方向的比例。 */
    float right_scale;     /* 右轮反馈和命令到模型正方向的比例。 */
    float T_limit;         /* 单轮力矩限幅，N*m。轮通道唯一限幅点。 */
    float T_to_I;          /* 轮力矩绝对方向到DJI原始电流的比例，已计入gear_ratio。 */
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
    float joint_T_limit;   /* 关节力矩限幅，N*m。关节通道唯一限幅点。 */
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
    float bench_L0_rate;     /* 板凳摇杆满杆的腿长调节速率，m/s。 */
    float bench_phi0_rate;   /* 板凳摇杆满杆的腿角调节速率，rad/s。 */
    float bench_L0_min;      /* 板凳可调腿长下限，m。 */
    float bench_L0_max;      /* 板凳可调腿长上限，m。 */
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
    float recover_theta;
    float L0_tol;
    float angle_tol;
    float stable_time;
    float prepare_timeout;
    float approach_timeout;
    float climb_timeout;
    float recover_timeout;
    /*
     * CLIMB 两段摆腿：先后摆蓄势再前摆越过台阶，最后归正。
     * phi0 为腿杆相对车体角，theta 为含pitch的腿摆绝对角，单位均为 rad。
     */
    float back_phi0_max;    /* 后摆腿杆角上限，超过改用PID保持。 */
    float back_phi0_hold;   /* 后摆超限后的PID保持角。 */
    float back_Tp;          /* 后摆虚拟腿摆力矩，N*m。 */
    float back_theta_exit;  /* 后摆转前摆的腿摆绝对角。 */
    float front_theta_max;  /* 前摆腿摆角上限，超过改用PID保持。 */
    float front_phi0_hold;  /* 前摆超限后的PID保持角。 */
    float front_Tp;         /* 前摆虚拟腿摆力矩，N*m。 */
    float front_theta_exit; /* 前摆转归正的腿摆绝对角。 */
    float home_phi0;        /* 归正目标腿杆角。 */
    /* 台阶模式姿态保护，比站立放宽，对应HERO_LEG磕台阶抬高倒地阈值。 */
    float pitch_limit;
    float phi0_min;
    float phi0_max;
    algorithm_pid_config_t leg_angle_pid;
} Chassis_Step_Config_t;

/**
 * @brief 整车实测机械与质量参数，控制前馈和观测统一从这里取。
 *
 * 每项注释同时记录MATLAB模型 ABK_LQR.m 的对应取值。两边不一致时，
 * 固件里的K不是本车的最优增益，必须重新生成后才谈得上最终整定。
 */
typedef struct
{
    float gravity;     /* 重力加速度，m/s^2。对应 g_ac。 */
    float body_mass;   /* 机体质量，kg。对应 m_b_ac。 */
    float leg_mass;    /* 单腿质量，kg。对应 m_l_ac。 */
    float wheel_mass;  /* 单轮质量，kg。对应 m_w_ac。 */
    float cg_to_hip;   /* 机体质心到腿部关节中心距离，m。对应 l_c_ac。 */
} Chassis_Model_Config_t;

/**
 * @brief 整车总质量，与MATLAB模型的质量分解保持一致。
 *
 * 重力前馈和全部力类观测阈值都以它为基准，只在这里算一次。
 */
static inline float Chassis_Model_Mass(const Chassis_Model_Config_t *config)
{
    // return config->body_mass +
    //        2.0f * config->leg_mass +
    //        2.0f * config->wheel_mass;
    return config->body_mass;
}

/** @brief 只用于Watch的打滑、离地、转向和卡腿观测参数。 */
typedef struct
{
    float residual_filter_s;
    float turn_filter_s;
    float normal_force_filter_s;
    /* 打滑：先过闸门再判进入，退出由起始轮速锁存决定，不再用退出计时。 */
    float slip_gate_yaw;      /* 闸门偏航残差阈值，rad/s。 */
    float slip_gate_v;        /* 闸门轮速与整车速度差阈值，m/s。 */
    float slip_v_enter;       /* 速度残差进入阈值，m/s。 */
    float slip_yaw_enter;     /* 偏航残差进入阈值，rad/s。 */
    float slip_dv_enter;      /* 轮速增量与加速度增量差进入阈值，m/s。 */
    float slip_enter_s;       /* 进入条件连续满足时间，s。 */
    /* 离地。 */
    float off_force_ratio;    /* 判离地的支撑力/标称静载比。 */
    float land_force_ratio;   /* 判落地的支撑力/标称静载比。 */
    float off_hold_s;
    float land_hold_s;
    float off_F_comp_ratio;   /* 整车离地后建议下压推力，相对单腿静载的比例。 */
    /* 转向。 */
    float turn_v_diff;        /* 触发转弯半径计算的左右轮速差阈值，m/s。 */
    float turn_force_limit_ratio;
    /* 卡腿。 */
    float stuck_T_ratio;      /* 判卡腿的轮力矩阈值，相对wheel.T_limit的比例。 */
    float stuck_theta_enter;  /* 判卡腿的腿摆角阈值，rad。 */
    float stuck_theta_exit;   /* 退出卡腿计时的腿摆角阈值，rad。 */
    float stuck_time;         /* 卡腿条件连续满足时间，s。 */
    float stuck_F0_coef_ratio;/* 补偿轴向力增长速率，相对单腿静载的比例每秒。 */
    float stuck_F0_max_ratio; /* 补偿轴向力上限，相对单腿静载的比例。 */
    float stuck_L0_coef;      /* 建议收腿量随卡腿时间增长的速率，m/s。 */
    float stuck_L0_max;       /* 建议收腿量上限，m。 */
} Chassis_Observer_Config_t;

/** @brief 双腿长poly22增益拟合的范围和系数表。 */
typedef struct
{
    float L0_min; /* 当前系数实际采样腿长下限，m。 */
    float L0_max; /* 当前系数实际采样腿长上限，m。 */
    /*
     * 十维状态各自的误差限幅，进K点乘之前生效，0表示该项不限幅。
     * 只限位置类状态，速度类保持不限，与参考工程一致。
     */
    float error_limit[CHASSIS_STATE_COUNT];
    /*
     * 展平成一维是为了能直接粘贴MATLAB输出，内存布局与[4][10][6]完全一致。
     * 下标 = (输出序号 * CHASSIS_STATE_COUNT + 状态序号) * 系数个数 + 系数序号。
     */
    float coefficients[CHASSIS_OUTPUT_COUNT * CHASSIS_STATE_COUNT *
                       ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT];
} Chassis_LQR_Config_t;

/** @brief 底盘控制唯一只读配置，集中保存机械、模型和安全参数。 */
typedef struct
{
    Chassis_Leg_Config_t leg[CHASSIS_LEG_COUNT];
    Chassis_Model_Config_t model;
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
    float F0_gravity_scale;           /* 重力前馈整定系数，1.0为按实测质量足额补偿。 */
    float F0_left;                    /* 左腿支撑力静态修正，N。 */
    float F0_right;                   /* 右腿支撑力静态修正，N。 */
    float default_dt;               /* 控制周期异常时采用的默认dt，s。 */
    float dt_min;                   /* 最小控制周期，s。 */
    float dt_max;                   /* 最大控制周期，s。 */
    float target[CHASSIS_STATE_COUNT]; /* 十维LQR默认目标状态。 */
} Chassis_Config_t;

extern const Chassis_Config_t Chassis_Config;

#ifdef __cplusplus
}
#endif

#endif
