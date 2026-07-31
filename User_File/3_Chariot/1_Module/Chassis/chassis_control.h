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
#define CHASSIS_FAULT_DISABLED 0x00000001UL       /* 非零输出许可关闭。 */
#define CHASSIS_FAULT_IMU 0x00000002UL            /* IMU未就绪或报错。 */
#define CHASSIS_FAULT_DM_MOTOR 0x00000004UL       /* 至少一个髋关节离线。 */
#define CHASSIS_FAULT_DJI_MOTOR 0x00000008UL      /* 至少一个轮电机离线。 */
#define CHASSIS_FAULT_CAN 0x00000010UL            /* CAN发送错误超过阈值。 */
#define CHASSIS_FAULT_CONTROL 0x00000020UL        /* 姿态、配置或计算保护。 */
#define CHASSIS_FAULT_KINEMATICS 0x00000040UL     /* 五连杆状态、逆解或力映射无效。 */
#define CHASSIS_FAULT_RECOVERY_TIMEOUT 0x00000080UL /* 恢复动作阶段超时。 */
#define CHASSIS_FAULT_REMOTE 0x00000100UL         /* 遥控器离线或收到急停请求。 */

/* 外部请求模式：表示操作者想让底盘执行的行为。 */
typedef enum
{
    CHASSIS_MODE_ZERO_FORCE = 0, /* 主动请求零力矩。 */
    CHASSIS_MODE_FOLLOW,         /* 正常平衡和跟随模式。 */
    CHASSIS_MODE_TOP,            /* 预留小陀螺模式。 */
    CHASSIS_MODE_SELF_SAVE,      /* 触发重新站立动作链。 */
    CHASSIS_MODE_BENCH,          /* 保持独立小板凳姿态。 */
} chassis_mode_t;

/* 内部执行状态：表示本周期任务应调用哪一条控制流程。 */
typedef enum
{
    CHASSIS_STANDING = 0,        /* 十维LQR和VMC站立控制。 */
    CHASSIS_ZERO_FORCE,          /* 最终关节力矩和轮电流清零。 */
    CHASSIS_FALLEN,              /* 倒地后的转腿阶段。 */
    CHASSIS_FALLING_TO_STAND,    /* 收腿到小板凳并等待稳定。 */
    CHASSIS_BENCH,               /* 独立板凳位置环和轮LQR。 */
} chassis_control_state_t;

typedef struct
{
    uint8_t initialized;         /* BMI088硬件初始化完成。 */
    uint8_t attitude_ready;      /* 零偏和姿态估计已经可用。 */
    uint32_t error_code;         /* IMU任务最近错误码。 */
    float roll_rad;              /* 整车右手系横滚角。 */
    float pitch_rad;             /* 整车右手系俯仰角。 */
    float yaw_rad;               /* 归一化偏航角。 */
    float yaw_total_rad;         /* 本次上电期间连续偏航角，单位 rad。 */
    float gyro_radps[APP_IMU_AXIS_COUNT];
    float motion_accel_mps2[APP_IMU_AXIS_COUNT];
} chassis_imu_t;

typedef struct
{
    uint8_t online;              /* 最近超时窗口内收到反馈。 */
    float position_rad;          /* 连续展开电机角，单位 rad。 */
    float speed_radps;           /* 电机角速度，单位 rad/s。 */
    float torque_nm;             /* 电机反馈力矩，单位 N*m。 */
} chassis_dm_motor_t;

typedef struct
{
    uint8_t online;              /* 最近超时窗口内收到反馈。 */
    int16_t speed_rpm;           /* DJI原始转速反馈，单位 rpm。 */
    int16_t current;             /* DJI原始电流反馈。 */
} chassis_dji_motor_t;

/** @brief 任务层发布给正常站立控制的遥控运动目标。 */
typedef struct
{
    float forward_speed_mps; /* 期望前进速度，单位 m/s。 */
    float yaw_target_rad;    /* 连续整车航向目标，单位 rad。 */
    float leg_length_m;      /* 左右对称目标腿长，单位 m。 */
    float yaw_anchor_rad;    /* 右摇杆航向偏置的连续锚点，单位 rad。 */
} chassis_motion_command_t;

typedef struct
{
    /* 外部意图和内部状态机。 */
    uint8_t enabled;               /* 非零电机输出许可，不控制 DM 协议使能。 */
    chassis_mode_t mode;
    chassis_mode_t last_mode;
    chassis_control_state_t state;

    /* 任务层每周期写入的设备反馈快照。 */
    chassis_imu_t imu;
    chassis_dm_motor_t dm_motor[APP_DM_COUNT];
    chassis_dji_motor_t wheel_motor[APP_WHEEL_COUNT];
    uint8_t remote_online;          /* 当前遥控输入后端处于在线状态。 */
    uint8_t remote_stop;            /* 急停请求，只封锁最终电机输出。 */
    uint8_t remote_control_ready;   /* 已上线且收到FOLLOW请求。 */
    uint8_t remote_target_valid;    /* 已建立过遥控目标，离线时保持腿长目标。 */
    uint8_t remote_self_save_latched; /* 阻止持续SELF_SAVE请求重复触发。 */
    uint32_t can_tx_error_count;
    float control_dt_s;            /* 底盘控制本轮实际周期，单位 s。 */

    chassis_motion_command_t motion_command;

    /* 五连杆、十维状态、LQR增益和运动融合中间量。 */
    chassis_vmc_state_t leg[CHASSIS_LEG_COUNT];
    float lqr_state[CHASSIS_STATE_COUNT];
    float target_state[CHASSIS_STATE_COUNT];
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

    /* 恢复、板凳和站立控制共同使用的目标与请求量。 */
    float state_elapsed_s;
    float state_stable_s;
    float target_leg_length_m[CHASSIS_LEG_COUNT];
    float target_leg_phi0_rad[CHASSIS_LEG_COUNT];
    float target_joint_angle_rad[APP_DM_COUNT];
    float target_joint_speed_radps[APP_DM_COUNT];
    float joint_torque_request_nm[APP_DM_COUNT]; /* 输出封锁时仍保留的关节请求。 */
    int16_t wheel_current_request[APP_WHEEL_COUNT]; /* LQR限幅后的轮电流请求。 */

    /* 只有这两组数组会进入任务层实际电机命令。 */
    float joint_torque_nm[APP_DM_COUNT];
    int16_t wheel_current[APP_WHEEL_COUNT];
    uint8_t safe_output;
    uint32_t fault_flags;
    uint8_t k_fit_enabled;
    uint8_t k_length_limited;
    uint8_t state_valid;           /* 本轮控制链已执行到末端，不代表输出已放行。 */

    /* 需要跨控制周期保存的滤波器和PID状态。 */
    algorithm_kalman_t speed_kalman;
    algorithm_pid_state_t leg_length_pid[CHASSIS_LEG_COUNT];
    algorithm_pid_state_t roll_pid;
    algorithm_pid_state_t joint_angle_pid[APP_DM_COUNT];
    algorithm_pid_state_t joint_speed_pid[APP_DM_COUNT];
} chassis_t;

/* 底盘唯一实际状态，也是 Watch 窗口的长期调试入口。 */
extern chassis_t chassis;

/**
 * @brief 初始化底盘控制状态、PID 和速度卡尔曼滤波器。
 */
void Chassis_ControlInit(void);

/**
 * @brief 清空速度融合和前进位移状态。
 */
void Chassis_ControlReset(void);

/**
 * @brief 由本轮原始关节反馈更新左右五连杆状态。
 */
void Chassis_ControlUpdateLegState(void);

/**
 * @brief 根据模式边沿和当前姿态选择控制状态并维护输出故障。
 */
void Chassis_ControlUpdateState(void);

/**
 * @brief 执行一轮十维 LQR、支撑力和 VMC 控制计算。
 */
void Chassis_ControlLoop(void);

/**
 * @brief 执行倒地转腿和小板凳准备两段式重新站立控制。
 */
void Chassis_RecoveryControlLoop(void);

/**
 * @brief 使用关节位置控制保持小板凳姿态，并由当前十维 LQR 控制双轮。
 */
void Chassis_BenchControlLoop(void);

/**
 * @brief 将底盘电机命令清零并重置运动融合状态。
 */
void Chassis_ZeroOutput(void);

#ifdef __cplusplus
}
#endif

#endif
