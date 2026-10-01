#ifndef SHOOTER_CONTROL_H
#define SHOOTER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "device_motor_dji.h"
#include "remote_input.h"
#include "shooter_config.h"

#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
#include "device_motor_lk.h"
#endif

#include <stdint.h>

/* 故障位，按台记（fric_fault[] / trigger_fault）。置位只清【那一台】的最终输出，不阻断请求量的计算。 */
#define SHOOTER_FAULT_NONE 0x00U
#define SHOOTER_FAULT_MOTOR 0x01U /* 电机离线（只对已使能总线上的电机计入）。 */
#define SHOOTER_FAULT_ESC 0x02U   /* 电机上报非零错误码（三相未接…；拨盘堵转交给卡弹状态机）。 */

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
 * ⚠ GIVEUP【故意不置 fault 位】：它只封锁拨盘的请求电流，摩擦轮照转——
 * 脱困后要能立刻接着打。
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
 * 和 safe_flag，由各自独立的任务驱动（shooter_task / gimbal_task），互相引用次数为 0。
 */
typedef struct
{
    float fric_target_rpm; /* 摩擦轮目标转速，rpm。减速箱已拆，即轮速。Watch 可改。 */
    /*
     * 拨盘力矩限幅，按电流给，A。Watch 可改，重启回到 Shooter_Config 的默认值。
     * 速度环输出之后硬限幅（刹车也受它约束），卡弹的"电流饱和"阈值按比例跟着它走。
     * 调它的目的：卡弹时拨盘顶不碎弹丸。
     */
    float trigger_I_limit;

    /*
     * 本任务自己取的遥控快照（Remote_Task_GetRemote），不借云台任务刷新的全局 Remote：
     * 云台任务一卡，发射部分就会拿着旧快照，急停也传不过来。
     */
    Remote_t remote;

    /*
     * 急停（SwC 在 UP 或 UNKNOWN）或遥控离线：两部分都要停，这是红线。
     * 判据与 gimbal_control.c 的 Gimbal_State_Update 同源。
     * ⚠ 发射部分怎么停：摩擦轮 PID 刹车（【不关】安全门，关了刹车电流就发不出去）；
     * 拨盘最终电流直接清零，不刹——理由见 Shooter_Command_Send。
     */
    uint8_t stop_flag;
    /*
     * 上膛锁：急停/离线期间置 1，解除后 VrB 必须先退出拨弹区才清 0，上电即为 1。
     * 防止一松急停就开火——那时摩擦轮刚从 0 起转，立刻拨弹还容易卡弹。只锁拨盘。
     */
    uint8_t trigger_lock;

    /* 由 VrB 派生的操作者意图。 */
    uint8_t fric_request;    /* VrB 过中位：摩擦轮开。 */
    uint8_t trigger_request; /* VrB 进使能区间：拨弹。 */

    motor_dji_state_t fric_motor[APP_DJI_FRIC_COUNT]; /* 摩擦轮反馈快照，每周期刷新。 */
    /*
     * 拨盘反馈快照，每周期刷新，类型随 APP_TRIGGER_MOTOR。两种状态的 angle（输出轴 rad）
     * 和 currentRaw（电调原始口径）同名同义，卡弹判据因此不用分支；
     * 转速字段不同名（speedRpm / speedDps），换算在 Shooter_Feedback_Update 里。
     */
#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
    motor_lk_state_t trigger_motor;
#else
    motor_dji_state_t trigger_motor;
#endif
    algorithm_pid_state_t fric_pid[APP_DJI_FRIC_COUNT];
    uint8_t fric_ready_flag; /* 三个摩擦轮都进了 fric_ready_rpm 带内。 */

    /*
     * 拨盘。⚠ 这一组全是【输出轴】口径（减速箱还在，36:1），
     * 与上面摩擦轮那套（减速箱已拆，转子即轮速）不同源。
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

    int16_t I_req[APP_DJI_FRIC_COUNT]; /* 摩擦轮请求电流；故障时仍然计算，供 Watch 观察。 */
    int16_t I[APP_DJI_FRIC_COUNT];     /* 摩擦轮安全门后实际下发的电流。 */
    int16_t trigger_I_req;             /* 拨盘请求电流，电调原始口径。 */
    int16_t trigger_I;                 /* 拨盘安全门后实际下发的电流。 */

    /*
     * ⚠ 发射部分【按台】隔离：哪台离线/报错只清它自己的最终命令，互不牵连——
     * 摩擦轮离线不停拨盘，拨盘离线也不停摩擦轮。离线那台必须清零：它的转速是旧值，
     * 拿它算刹车就等于一个固定电流。
     */
    uint32_t fric_fault[APP_DJI_FRIC_COUNT]; /* 每台摩擦轮的故障位，见 SHOOTER_FAULT_*。 */
    uint32_t trigger_fault;                  /* 拨盘的故障位。 */
    uint32_t fault;    /* 上面几台的按位或，只作 Watch 总览，不参与清零。 */
    uint8_t safe_flag; /* 置位表示发射部分整体输出零；现在只由 APP_GIMBAL_OUTPUT_ENABLE 决定。 */
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
