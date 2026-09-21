#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device_motor_dji.h"
#include "device_motor_dm.h"
#include "gimbal_config.h"
#include "remote_input.h"
#include "task_imu.h"

#include <stdint.h>

/* 故障位。任一置位都会封锁两路输出，但不阻断请求量的计算。 */
#define GIMBAL_FAULT_NONE 0x00U
#define GIMBAL_FAULT_DISABLED 0x01U /* 输出开关关闭。 */
#define GIMBAL_FAULT_IMU 0x02U      /* IMU 未初始化或姿态未就绪。 */
#define GIMBAL_FAULT_DM 0x04U       /* 云台 DM 电机离线。 */
#define GIMBAL_FAULT_DJI 0x08U      /* 发射机构 DJI 电机离线。 */
#define GIMBAL_FAULT_CAN 0x10U      /* CAN 发送错误累计超限。 */
#define GIMBAL_FAULT_REMOTE 0x100U  /* 遥控离线。 */

/** @brief 云台外层状态，由右拨杆直接决定。 */
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, /* 卸力，两路输出恒零。 */
    GIMBAL_CONTROL,        /* 遥控控制。 */
} Gimbal_State_t;

/** @brief 单个云台轴的运行状态。 */
typedef struct
{
    float angle;          /* IMU 口径角度，rad，受控量。 */
    float d_angle;        /* IMU 口径角速度，rad/s；YAW 已做俯仰投影。 */
    float angle_joint;    /* 编码器口径关节角，rad；用于机械限位和 YAW 投影。 */
    float d_angle_joint;  /* 编码器口径关节角速度，rad/s；仅诊断。 */
    float T_fb;           /* 电机反馈力矩，N*m。 */
    float target;         /* 角度目标，rad，与 angle 同口径。 */
    float d_angle_target; /* 角度环输出的目标角速度，rad/s。 */
    float T_req;          /* 控制器请求力矩，N*m；故障时仍然计算。 */
    float T;              /* 安全门后实际下发的力矩，N*m。 */
    uint8_t online;       /* 电机在线。 */
    algorithm_pid_state_t angle_pid; /* 外环状态。 */
    algorithm_pid_state_t speed_pid; /* 内环状态。 */
} Gimbal_Axis_t;

/** @brief 整机唯一运行状态，也是长期 Watch 入口。 */
typedef struct
{
    Gimbal_State_t state;        /* 当前外层状态。 */
    uint32_t fault;              /* 故障位集合，见 GIMBAL_FAULT_*。 */
    float dt;                    /* 本周期实际时长，单位 s。 */
    Gimbal_Axis_t yaw;
    Gimbal_Axis_t pitch;
    task_imu_state_t imu;        /* IMU 快照，经 IMU_Task_GetState 获取。 */
    int16_t I_dji_req[APP_DJI_COUNT]; /* 发射机构请求电流；本轮恒零。 */
    int16_t I_dji[APP_DJI_COUNT];     /* 安全门后实际下发电流。 */
    uint8_t pitch_home_flag;     /* 使能后 PITCH 正在回水平，期间不吃摇杆。 */
    uint8_t safe_flag;           /* 置位表示本周期必须输出零。 */
} Gimbal_t;

extern Gimbal_t Gimbal;

void Gimbal_Init(void);
void Gimbal_Feedback_Update(void);
void Gimbal_State_Update(void);
void Gimbal_Control(void);
void Gimbal_Command_Send(void);

#ifdef __cplusplus
}
#endif

#endif
