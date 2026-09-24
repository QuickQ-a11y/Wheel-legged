#ifndef SHOOTER_CONTROL_H
#define SHOOTER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device_motor_dji.h"
#include "shooter_config.h"

#include <stdint.h>

/* 故障位。任一置位都会封锁发射机构的电流输出，但不阻断请求量的计算。 */
#define SHOOTER_FAULT_NONE 0x00U
#define SHOOTER_FAULT_MOTOR 0x01U /* 已使能的那条总线上有电机离线。 */
#define SHOOTER_FAULT_ESC 0x02U   /* 有电机上报非零错误码（堵转、三相未接…）。 */

/**
 * @brief 发射机构唯一运行状态，也是长期 Watch 入口。
 *
 * ⚠ 与云台【互不封锁】：一台摩擦轮掉线不该让操作者连瞄准都做不了，
 * 反过来云台故障也不该停下已经转起来的摩擦轮。两个模块各有自己的 fault
 * 和 safe_flag，由 gimbal_task 并列调用。
 */
typedef struct
{
    /*
     * 本步的控制入口，都在 Watch 里手改。
     * VrB 旋钮的接入是下一步——速度环整定期间要能随时改目标转速，
     * 也不希望旋钮误碰就起转。
     */
    uint8_t enable_flag;      /* 总开关，为 0 时请求电流恒零。 */
    float fric_target_rpm;    /* 摩擦轮目标【转子】转速，rpm。 */
    int16_t trigger_I_manual; /* 拨盘手动电流，本步唯一的拨盘控制手段。 */

    /* 由遥控派生，本步【只产出、不消费】，供 Watch 核对 VrB 解得对不对。 */
    uint8_t fric_request;    /* VrB 过中位。 */
    uint8_t trigger_request; /* VrB 进使能区间。 */

    motor_dji_state_t motor[APP_DJI_COUNT]; /* 反馈快照，任务层每周期刷新。 */
    algorithm_pid_state_t fric_pid[APP_DJI_FRIC_COUNT];
    uint8_t fric_ready_flag; /* 三个摩擦轮都进了 fric_ready_rpm 带内。 */

    int16_t I_req[APP_DJI_COUNT]; /* 请求电流；故障时仍然计算，供 Watch 观察。 */
    int16_t I[APP_DJI_COUNT];     /* 安全门后实际下发的电流。 */

    uint32_t fault;    /* 故障位集合，见 SHOOTER_FAULT_*。 */
    uint8_t safe_flag; /* 置位表示本周期必须输出零。 */
    float dt;          /* 本周期实际时长，单位 s。 */
} Shooter_t;

extern Shooter_t Shooter;

void Shooter_Init(void);
void Shooter_Feedback_Update(void);
void Shooter_Control(void);
void Shooter_Command_Send(void);

#ifdef __cplusplus
}
#endif

#endif
