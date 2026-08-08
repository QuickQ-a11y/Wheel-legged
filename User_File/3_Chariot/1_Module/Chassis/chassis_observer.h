#ifndef CHASSIS_OBSERVER_H
#define CHASSIS_OBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_config.h"
#include "chassis_vmc.h"

#include <stdint.h>

/*
 * 打滑、离地、转向和卡腿是四套互不依赖的只读观测，各自拥有一份状态。
 * 四者都按 Init 建立初值、Update 计算观测量、Calc 给出结论的顺序使用，
 * 全部只写自己的结构体，不回写任何控制量。
 *
 * 判定口径参考 HERO_LEG 实机工程：轮速残差闸门与打滑锁存退出、整车离地
 * 排除倒地和台阶、左右轮速差反推转弯半径、轮力矩与腿角双条件判卡腿。
 * 阈值一律按本车量级重新给保守初值，未沿用原车数值。
 */

/** @brief 轮速与整车偏航残差得到的打滑观测状态。 */
typedef struct
{
    uint8_t init_flag;                    /* 已建立上一周期速度基准。 */
    uint8_t gate_flag;                    /* 满足打滑判定闸门且当前不是小陀螺。 */
    float d_fai_wheel;                    /* 左右轮速反推的偏航角速度，rad/s。 */
    float yaw_res;                        /* 轮速偏航减IMU偏航，rad/s。 */
    float yaw_res_lpf;                    /* 偏航残差低通值，rad/s。 */
    float dv_acc;                         /* IMU前向加速度积分的单周期速度增量，m/s。 */
    float v_wheel[CHASSIS_LEG_COUNT];     /* 轮缘线速度，m/s。 */
    float dv_wheel[CHASSIS_LEG_COUNT];    /* 轮缘线速度单周期增量低通值，m/s。 */
    float v_expect[CHASSIS_LEG_COUNT];    /* 整车速度和偏航推出的单侧期望速度，m/s。 */
    float v_res[CHASSIS_LEG_COUNT];       /* 实际单侧轮腿速度减期望速度，m/s。 */
    float v_res_lpf[CHASSIS_LEG_COUNT];   /* 速度残差低通值，m/s。 */
    float dv_res[CHASSIS_LEG_COUNT];      /* 轮速增量减加速度积分增量，m/s。 */
    float v_latch[CHASSIS_LEG_COUNT];     /* 打滑起始锁存的轮缘线速度，退出判定用，m/s。 */
    float enter_time[CHASSIS_LEG_COUNT];  /* 进入条件连续满足时间，s。 */
    uint8_t candidate_flag[CHASSIS_LEG_COUNT]; /* 本轮满足打滑进入阈值。 */
    uint8_t slip_flag[CHASSIS_LEG_COUNT];      /* 判定后的单侧打滑结论。 */
    float last_v[CHASSIS_LEG_COUNT];      /* 上一周期单侧轮腿速度，m/s。 */
    float last_v_wheel[CHASSIS_LEG_COUNT];/* 上一周期轮缘线速度，m/s。 */
} Chassis_Slip_t;

/** @brief 关节反馈力矩反解支撑力得到的离地观测状态。 */
typedef struct
{
    uint8_t init_flag;                          /* 已建立上一周期腿速度基准。 */
    VMC_Force_t force[CHASSIS_LEG_COUNT];       /* 关节反馈力矩反解的虚拟腿广义力。 */
    uint8_t valid_flag[CHASSIS_LEG_COUNT];      /* 本轮支撑力估计有效。 */
    float dd_L0[CHASSIS_LEG_COUNT];             /* 腿长二阶导低通值，m/s^2。 */
    float dd_theta[CHASSIS_LEG_COUNT];          /* 腿摆角二阶导低通值，rad/s^2。 */
    float Fn_raw[CHASSIS_LEG_COUNT];            /* 单腿支撑力瞬时值，N。 */
    float Fn[CHASSIS_LEG_COUNT];                /* 单腿支撑力低通值，N。 */
    float Fn_ratio[CHASSIS_LEG_COUNT];          /* 支撑力与标称单腿静载之比。 */
    uint8_t Fn_init_flag[CHASSIS_LEG_COUNT];    /* 支撑力低通已用首个有效值建立初值。 */
    float Fn_static;                            /* 标称模型单腿静载，N。 */
    float fn_comp;                              /* 整车离地后建议的向下补偿推力，N；本轮不接入F0。 */
    float off_time[CHASSIS_LEG_COUNT];          /* 离地条件连续满足时间，s。 */
    float land_time[CHASSIS_LEG_COUNT];         /* 落地条件连续满足时间，s。 */
    uint8_t off_candidate_flag[CHASSIS_LEG_COUNT]; /* 本轮支撑力低于离地阈值。 */
    uint8_t off_ground_flag[CHASSIS_LEG_COUNT];    /* 判定后的单腿离地结论。 */
    uint8_t all_off_flag;                       /* 左右腿同时离地，且不处于倒地或台阶动作。 */
    float last_d_L0[CHASSIS_LEG_COUNT];         /* 上一周期腿长速度，m/s。 */
    float last_d_theta[CHASSIS_LEG_COUNT];      /* 上一周期腿摆角速度，rad/s。 */
} Chassis_Ground_t;

/** @brief 转弯半径、离心修正和左右支撑力差的转向观测状态。 */
typedef struct
{
    float a_y_imu;       /* IMU横向加速度，m/s^2。 */
    float a_y_kin;       /* 速度与偏航角速度推出的向心加速度，m/s^2。 */
    float a_y_imu_lpf;   /* IMU横向加速度低通值，m/s^2。 */
    float a_y_kin_lpf;   /* 运动学向心加速度低通值，m/s^2。 */
    float a_y_res;       /* 两路横向加速度之差，m/s^2。 */
    float R_turn;        /* 二轮差速模型转弯半径，m；直行或打滑时为0。 */
    float dd_s_turn;     /* 转弯离心修正项 d_fai^2 * R_turn，m/s^2。 */
    float dd_s_fix;      /* 离心修正后的前向加速度，m/s^2；本轮不喂速度Kalman。 */
    float h_cg;          /* 标称模型质心高度，m。 */
    float dF_imu;        /* IMU路左右支撑力差，N。 */
    float dF_kin;        /* 运动学路左右支撑力差，N。 */
    float dF_imu_lim;    /* 按标称静载限幅后的IMU路支撑力差，N。 */
    float dF_kin_lim;    /* 按标称静载限幅后的运动学路支撑力差，N。 */
} Chassis_Turn_t;

/** @brief 轮力矩与腿摆角双条件得到的卡腿观测状态。 */
typedef struct
{
    float T_wheel[CHASSIS_LEG_COUNT];      /* 本轮单侧轮力矩绝对值，N*m。 */
    float theta[CHASSIS_LEG_COUNT];        /* 本轮单侧腿摆角绝对值，rad。 */
    float stuck_time[CHASSIS_LEG_COUNT];   /* 卡腿条件连续满足时间，s。 */
    uint8_t stuck_flag[CHASSIS_LEG_COUNT]; /* 判定后的单腿卡腿结论。 */
    float comp_F0[CHASSIS_LEG_COUNT];      /* 建议补偿轴向力，N；本轮不接入F0。 */
    float comp_L0[CHASSIS_LEG_COUNT];      /* 建议收腿量，m；本轮不接入目标腿长。 */
} Chassis_Stuck_t;

typedef struct Chassis Chassis_t;

/**
 * @brief 清空打滑观测状态。
 */
void Chassis_Slip_Init(Chassis_Slip_t *slip);

/**
 * @brief 计算偏航残差、单侧速度残差和轮速增量，并保存本轮差分基准。
 */
void Chassis_Slip_Update(const Chassis_Config_t *config, Chassis_t *chassis);

/**
 * @brief 闸门成立时由残差阈值判定单侧打滑，退出用起始轮速锁存。
 */
void Chassis_Slip_Calc(const Chassis_Config_t *config, Chassis_t *chassis);

/**
 * @brief 清空离地观测状态。
 */
void Chassis_Ground_Init(Chassis_Ground_t *ground);

/**
 * @brief 反解虚拟腿广义力并估算单腿支撑力及其标称静载比。
 */
void Chassis_Ground_Update(const Chassis_Config_t *config, Chassis_t *chassis);

/**
 * @brief 由支撑力比例和离地、落地双时间迟滞判定单腿离地。
 */
void Chassis_Ground_Calc(const Chassis_Config_t *config, Chassis_t *chassis);

/**
 * @brief 清空转向观测状态。
 */
void Chassis_Turn_Init(Chassis_Turn_t *turn);

/**
 * @brief 计算IMU和运动学两路横向加速度及标称质心高度。
 */
void Chassis_Turn_Update(const Chassis_Config_t *config, Chassis_t *chassis);

/**
 * @brief 由左右轮速差反推转弯半径和离心修正，并估算左右支撑力差。
 */
void Chassis_Turn_Calc(const Chassis_Config_t *config, Chassis_t *chassis);

/**
 * @brief 清空卡腿观测状态。
 */
void Chassis_Stuck_Init(Chassis_Stuck_t *stuck);

/**
 * @brief 取本轮轮力矩和腿摆角绝对值作为卡腿判定输入。
 */
void Chassis_Stuck_Update(const Chassis_Config_t *config, Chassis_t *chassis);

/**
 * @brief 轮力矩和腿摆角同时超阈值并持续后判定卡腿，给出补偿建议值。
 */
void Chassis_Stuck_Calc(const Chassis_Config_t *config, Chassis_t *chassis);

#ifdef __cplusplus
}
#endif

#endif
