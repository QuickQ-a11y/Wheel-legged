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

/*
 * 拨弹状态机。语义由 app_config.h 的 APP_SHOOT_BURST_* 注释块定义：
 * 进使能区立刻一发 -> 保持超过 BURST_DELAY 转连发 -> 退出即复位。
 *
 * ⚠ "那一发"发生在 IDLE -> HOLD 的【转移】上，没有单独的 SINGLE 状态：
 * 一次进入使能区只会转移一次，所以"一次进入 = 一发"天然成立，
 * 不需要在遥控层做边沿检测，也不需要一个只存在一个周期的状态。
 */
typedef enum
{
    SHOOTER_TRIGGER_IDLE = 0, /* VrB 未进使能区；目标角跟随当前角。 */
    SHOOTER_TRIGGER_HOLD,     /* 已打一发，等着看要不要转连发。 */
    SHOOTER_TRIGGER_BURST,    /* 连发，按 APP_SHOOT_BURST_HZ 持续加发。 */
} shooter_trigger_state_t;

/*
 * 防卡弹状态机，与拨弹状态机【正交且优先】：
 * BACKING / GIVEUP 期间拨弹逻辑整个被旁路，退弹指令不会被下一轮覆盖。
 *
 * ⚠ 判据只在 NORMAL 判。退弹本身也会顶出大电流，别的状态里再判会自锁。
 *
 * ⚠ GIVEUP【故意不置 fault 位】：fault 会经 safe_flag 把四台电机全部清零，
 * 而拨盘卡死时摩擦轮应该继续转——脱困后要能立刻接着打。
 * 放弃这件事本身就由这个状态表示，不另存一份。
 */
typedef enum
{
    SHOOTER_JAM_NORMAL = 0, /* 正常。 */
    SHOOTER_JAM_SUSPECT,    /* 判据成立，还没到确认时长。 */
    SHOOTER_JAM_BACKING,    /* 正在退弹。 */
    SHOOTER_JAM_GIVEUP,     /* 连续退弹无效，放弃；只能靠 VrB 退出使能区解除。 */
} shooter_jam_state_t;

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
    uint8_t enable_flag;   /* 总开关，为 0 时请求电流恒零。 */
    float fric_target_rpm; /* 摩擦轮目标【转子】转速，rpm。Watch 可改。 */

    /* 由 VrB 派生的操作者意图。 */
    uint8_t fric_request;    /* VrB 过中位：摩擦轮开。 */
    uint8_t trigger_request; /* VrB 进使能区间：拨弹。 */

    motor_dji_state_t motor[APP_DJI_COUNT]; /* 反馈快照，任务层每周期刷新。 */
    algorithm_pid_state_t fric_pid[APP_DJI_FRIC_COUNT];
    uint8_t fric_ready_flag; /* 三个摩擦轮都进了 fric_ready_rpm 带内。 */

    /*
     * 拨盘。⚠ 这一组全是【输出轴】口径，与上面摩擦轮的【转子】rpm 不同源。
     * 一发 = 2π/N 只在输出轴上成立，所以拨盘这边整体换到输出轴。
     */
    shooter_trigger_state_t trigger_state;
    shooter_jam_state_t jam_state;
    float trigger_target;       /* 目标角，输出轴 rad。累加量，不是绝对位置。 */
    float trigger_angle;        /* 反馈角，输出轴 rad。 */
    float trigger_omega;        /* 反馈角速度，输出轴 rad/s。由 speedRpm 换算。 */
    float trigger_target_omega; /* 位置环输出 = 速度环目标，输出轴 rad/s。 */
    algorithm_pid_state_t trigger_angle_pid;
    algorithm_pid_state_t trigger_speed_pid;
    float trigger_state_s; /* 在当前拨弹状态停留的时长，s。 */
    float jam_state_s;     /* 在当前卡弹状态停留的时长，s。 */
    float burst_s;         /* 连发节拍计时，s。 */
    uint32_t jam_retry;    /* 本轮连续退弹次数，回到 IDLE 时清零。 */

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
