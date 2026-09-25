#include "shooter_task.h"

#include "app_config.h"
#include "shooter_control.h"

#include "cmsis_os2.h"

static osThreadId_t shooterTaskHandle;

static const osThreadAttr_t shooterTaskAttributes = {
    .name = "ShooterTask",
    .stack_size = 1024U * 4U,
    .priority = (osPriority_t)osPriorityHigh,
};

/**
 * @brief 发射机构主任务，1 kHz 固定周期。
 *
 * 每轮按 反馈更新 -> 控制 -> 命令发送 执行；
 * 设备故障只在命令发送阶段覆盖零电流，不阻断前面的计算。
 *
 * ⚠ 与云台任务【同优先级并列】，各自 osDelayUntil 独立相位，互不阻塞。
 * 给同级而不是低一级，是因为两者都是 1 kHz 硬周期：低一级会被
 * AboveNormal 的遥控/USB 任务抢占，拨盘位置环对周期抖动比速度环敏感。
 *
 * 拆成两个任务不引入新的共享状态：云台只碰 Motor_DM、发射机构只碰 Motor_DJI，
 * 而 CAN 发送缓存由 task_can 的 canTxMutex 保护。
 * dt 各自独立测量，不再借云台那一份。
 */
static void Shooter_Task_Entry(void *argument)
{
    uint32_t wakeTick = osKernelGetTickCount();
    uint32_t lastTick = wakeTick;

    (void)argument;

    Shooter_Init();

    for (;;)
    {
        wakeTick += APP_CTRL_TICKS;
        osDelayUntil(wakeTick);

        uint32_t nowTick = osKernelGetTickCount();
        float dt = (float)(nowTick - lastTick) * 0.001f;

        /* tick 差越界说明被长时间抢占或计数回绕，退回标称周期。 */
        Shooter.dt = ((dt > 0.0f) && (dt < 0.05f)) ? dt : Shooter_Config.default_dt;
        lastTick = nowTick;

        Shooter_Feedback_Update();
        Shooter_Control();
        Shooter_Command_Send();
    }
}

void Shooter_Task_Init(void)
{
    shooterTaskHandle = osThreadNew(Shooter_Task_Entry, NULL, &shooterTaskAttributes);
}
