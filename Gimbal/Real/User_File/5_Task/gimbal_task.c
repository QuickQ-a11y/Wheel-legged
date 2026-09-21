#include "gimbal_task.h"

#include "app_config.h"
#include "gimbal_control.h"

#include "cmsis_os2.h"

static osThreadId_t gimbalTaskHandle;

static const osThreadAttr_t gimbalTaskAttributes = {
    .name = "GimbalTask",
    .stack_size = 2048U * 4U,
    .priority = (osPriority_t)osPriorityHigh,
};

/**
 * @brief 云台主任务，1 kHz 固定周期。
 *
 * 每轮按 反馈更新 -> 状态选择 -> 控制 -> 命令发送 执行；
 * 设备故障只在命令发送阶段覆盖零力矩和零电流，不阻断前面的计算。
 */
static void Gimbal_Task_Entry(void *argument)
{
    uint32_t wakeTick = osKernelGetTickCount();
    uint32_t lastTick = wakeTick;

    (void)argument;

    Gimbal_Init();

    for (;;)
    {
        wakeTick += APP_CTRL_TICKS;
        osDelayUntil(wakeTick);

        uint32_t nowTick = osKernelGetTickCount();
        float dt = (float)(nowTick - lastTick) * 0.001f;

        /* tick 差越界说明被长时间抢占或计数回绕，退回标称周期。 */
        Gimbal.dt = ((dt > 0.0f) && (dt < 0.05f)) ? dt : Gimbal_Config.default_dt;
        lastTick = nowTick;

        Gimbal_Feedback_Update();
        Gimbal_State_Update();
        Gimbal_Control();
        Gimbal_Command_Send();
    }
}

void Gimbal_Task_Init(void)
{
    gimbalTaskHandle = osThreadNew(Gimbal_Task_Entry, NULL, &gimbalTaskAttributes);
}
