#include "chassis_remote.h"

#include "app_config.h"

#include <stddef.h>

static remote_mode_request_t previous_mode = REMOTE_MODE_NONE;
static uint8_t yaw_stick_flag;

/** @brief 取得十维模型使用的连续整车航向。 */
static float Model_Yaw_Get(void)
{
    return Chassis.imu.yaw_total * Chassis_Config.imu.yaw_angle_scale;
}

/** @brief 急停或离线时清零运动命令并从当前连续航向重新锚定。 */
static void Remote_Hold(void)
{
    float yaw = Model_Yaw_Get();

    Chassis.goal.d_s = 0.0f;
    Chassis.goal.d_y = 0.0f;
    Chassis.goal.d_fai = 0.0f;
    Chassis.goal.fai_anchor = yaw;
    Chassis.goal.fai = yaw;
    yaw_stick_flag = 0U;
}

/** @brief 将通用遥控输入转换为底盘物理运动目标。 */
static void Remote_Goal_Update(const Remote_t *remote)
{
    float yaw_axis = remote->rightStick.x;
    float yaw = Model_Yaw_Get();

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
        Chassis.goal.fai_anchor = yaw;
        Chassis.goal.fai = yaw;
        yaw_stick_flag = 0U;
    }
    else
    {
        Chassis.goal.d_s =
            remote->leftStick.y * APP_RC_MAX_VEL;
        Chassis.goal.d_y = 0.0f;
        Chassis.goal.d_fai = 0.0f;

        if (yaw_axis != 0.0f)
        {
            if (yaw_stick_flag == 0U)
            {
                Chassis.goal.fai_anchor = yaw;
            }
            yaw_stick_flag = 1U;
            Chassis.goal.fai =
                Chassis.goal.fai_anchor -
                yaw_axis * APP_RC_MAX_YAW;
        }
        else
        {
            if (yaw_stick_flag != 0U)
            {
                Chassis.goal.fai_anchor = yaw;
            }
            yaw_stick_flag = 0U;
            Chassis.goal.fai =
                Chassis.goal.fai_anchor;
        }
    }

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

void Chassis_Remote_Init(void)
{
    previous_mode = REMOTE_MODE_NONE;
    yaw_stick_flag = 0U;
}

void Chassis_Remote_Update(const Remote_t *remote)
{
    remote_mode_request_t mode_request;

    if (remote == NULL)
    {
        mode_request = REMOTE_MODE_NONE;
    }
    else
    {
        mode_request = remote->modeRequest;
    }

    Chassis.remote_online_flag =
        ((remote != NULL) && (remote->online != 0U)) ? 1U : 0U;
    Chassis.remote_stop_flag =
        ((remote != NULL) &&
         (remote->rightSwitch == REMOTE_SWITCH_DOWN)) ? 1U : 0U;

    if ((Chassis.remote_online_flag == 0U) || (Chassis.remote_stop_flag != 0U))
    {
        Chassis.enable_flag = 0U;
        Chassis.remote_ready_flag = 0U;
        Remote_Hold();
        previous_mode = mode_request;
        return;
    }

    if (Chassis.remote_ready_flag == 0U)
    {
        Chassis.enable_flag = 0U;
        Remote_Hold();
        if (mode_request != REMOTE_MODE_FOLLOW)
        {
            previous_mode = mode_request;
            return;
        }

        Chassis.remote_ready_flag = 1U;
        Chassis.remote_target_flag = 1U;
        Chassis.goal.fai_anchor = Model_Yaw_Get();
        Chassis.goal.fai =
            Chassis.goal.fai_anchor;
        if (Chassis.mode != CHASSIS_MODE_SELF_SAVE)
        {
            Chassis.mode = CHASSIS_MODE_FOLLOW;
        }
    }

    Chassis.enable_flag = 1U;
    Remote_Goal_Update(remote);

    /* 只有恢复状态机真正回到STANDING后，才自动结束SELF_SAVE请求。 */
    if ((Chassis.mode == CHASSIS_MODE_SELF_SAVE) &&
        (Chassis.last_mode == CHASSIS_MODE_SELF_SAVE) &&
        (Chassis.state == CHASSIS_STANDING))
    {
        Chassis.mode = CHASSIS_MODE_FOLLOW;
    }

    if (Chassis.mode == CHASSIS_MODE_SELF_SAVE)
    {
        previous_mode = mode_request;
        return;
    }

    /* 自救结束后保持FOLLOW，直到重新收到FOLLOW请求才解除触发锁。 */
    if (Chassis.recovery_latch_flag != 0U)
    {
        Chassis.mode = CHASSIS_MODE_FOLLOW;
        if (mode_request == REMOTE_MODE_FOLLOW)
        {
            Chassis.recovery_latch_flag = 0U;
        }
        previous_mode = mode_request;
        return;
    }

    switch (mode_request)
    {
    case REMOTE_MODE_FOLLOW:
        Chassis.mode = CHASSIS_MODE_FOLLOW;
        break;

    case REMOTE_MODE_BENCH:
        Chassis.mode = CHASSIS_MODE_BENCH;
        break;

    case REMOTE_MODE_TOP:
        Chassis.mode = CHASSIS_MODE_TOP;
        break;

    case REMOTE_MODE_STEP:
        Chassis.mode = CHASSIS_MODE_STEP;
        break;

    case REMOTE_MODE_SELF_SAVE:
        if ((previous_mode != REMOTE_MODE_SELF_SAVE) &&
            (previous_mode != REMOTE_MODE_NONE))
        {
            Chassis.recovery_latch_flag = 1U;
            Chassis.mode = CHASSIS_MODE_SELF_SAVE;
        }
        break;

    case REMOTE_MODE_NONE:
    default:
        break;
    }

    previous_mode = mode_request;
}
