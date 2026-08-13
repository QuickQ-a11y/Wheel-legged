#include "chassis_remote.h"

#include "app_config.h"

#include <string.h>

/** @brief 急停或离线时清零全部运动命令，航向由控制层保持在当前值。 */
static void Remote_Hold(void)
{
    Chassis.goal.d_s = 0.0f;
    Chassis.goal.d_y = 0.0f;
    Chassis.goal.d_fai = 0.0f;
    Chassis.yaw_stick_flag = 0U;
    memset(Chassis.goal.bench_d_L0, 0, sizeof(Chassis.goal.bench_d_L0));
    memset(Chassis.goal.bench_d_phi0, 0, sizeof(Chassis.goal.bench_d_phi0));
}

/** @brief 将通用遥控输入转换为底盘物理运动目标。 */
static void Remote_Goal_Update(const Remote_t *remote)
{
    float yaw_axis = remote->rightStick.x;

    if (remote->modeRequest == REMOTE_MODE_TOP)
    {
        Chassis.goal.d_s =
            remote->leftStick.y *
            Chassis_Config.top.max_d_s;
        Chassis.goal.d_y =
            remote->leftStick.x *
            Chassis_Config.top.max_d_s;
        Chassis.goal.d_fai =
            -yaw_axis * Chassis_Config.top.max_d_fai;
    }
    else
    {
        Chassis.goal.d_s =
            remote->leftStick.y * APP_RC_MAX_VEL;
        Chassis.goal.d_y = 0.0f;
        /* 右摇杆给偏航角速度；航向目标由控制层积分并在松杆时锁存。 */
        Chassis.goal.d_fai = -yaw_axis * APP_RC_MAX_YAW;
    }

    /* 板凳模式用左右摇杆分别微调两条腿，控制层按本速率积分目标。 */
    Chassis.goal.bench_d_L0[CHASSIS_LEFT] =
        remote->leftStick.y * Chassis_Config.recovery.bench_L0_rate;
    Chassis.goal.bench_d_L0[CHASSIS_RIGHT] =
        remote->rightStick.y * Chassis_Config.recovery.bench_L0_rate;
    Chassis.goal.bench_d_phi0[CHASSIS_LEFT] =
        remote->leftStick.x * Chassis_Config.recovery.bench_phi0_rate;
    Chassis.goal.bench_d_phi0[CHASSIS_RIGHT] =
        remote->rightStick.x * Chassis_Config.recovery.bench_phi0_rate;

    switch (remote->legRequest)
    {
    case REMOTE_LEG_SHORT:
        Chassis.goal.L0 = APP_RC_LEG_S;
        break;

    case REMOTE_LEG_MIDDLE:
        Chassis.goal.L0 = APP_RC_LEG_M;
        break;

    case REMOTE_LEG_LONG:
        Chassis.goal.L0 = APP_RC_LEG_L;
        break;

    case REMOTE_LEG_KEEP:
    default:
        break;
    }
}

/**
 * @brief 将通用遥控输入转换为底盘运动目标和外层模式。
 *
 * 模式直接跟随拨杆档位，不做边沿检测和锁存：Chassis.mode 永远等于操作者
 * 当前的拨杆位置，任何状态下拨一下就能改，因此不存在模式卡死。
 * 离线或急停只关闭输出许可并回中运动目标，模式保持不变，控制中间量继续计算。
 */
void Chassis_Remote_Update(const Remote_t *remote)
{
    Chassis.remote_online_flag = (remote->online != 0U) ? 1U : 0U;
    Chassis.remote_stop_flag =
        (remote->rightSwitch == REMOTE_SWITCH_DOWN) ? 1U : 0U;

    if ((Chassis.remote_online_flag == 0U) || (Chassis.remote_stop_flag != 0U))
    {
        Chassis.enable_flag = 0U;
        Remote_Hold();
        return;
    }

    Chassis.enable_flag = 1U;
    Remote_Goal_Update(remote);

    switch (remote->modeRequest)
    {
    case REMOTE_MODE_ZERO_FORCE:
        Chassis.mode = CHASSIS_MODE_ZERO_FORCE;
        break;

    case REMOTE_MODE_FOLLOW:
        Chassis.mode = CHASSIS_MODE_FOLLOW;
        break;

    case REMOTE_MODE_BENCH:
        Chassis.mode = CHASSIS_MODE_BENCH;
        break;

    case REMOTE_MODE_SELF_SAVE:
        Chassis.mode = CHASSIS_MODE_SELF_SAVE;
        break;

    case REMOTE_MODE_TOP:
        Chassis.mode = CHASSIS_MODE_TOP;
        break;

    case REMOTE_MODE_STEP:
        Chassis.mode = CHASSIS_MODE_STEP;
        break;

    case REMOTE_MODE_NONE:
    default:
        break;
    }
}
