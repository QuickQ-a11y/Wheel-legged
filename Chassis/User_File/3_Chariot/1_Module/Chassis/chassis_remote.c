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

/**
 * @brief VrA 选模式组，双阈值滞回。
 *
 * 中间带里保持当前组，所以旋钮停在中值附近时不会反复横跳。
 * 用户要的是"随时可切组"，因此这里不检查 SwA/SwB 是否归位。
 */
static void Remote_Group_Update(const Remote_t *remote)
{
    if (remote->knobA > APP_RC_KNOB_HYST)
    {
        Chassis.knob_group = 1U;
    }
    else if (remote->knobA < -APP_RC_KNOB_HYST)
    {
        Chassis.knob_group = 0U;
    }
}

/**
 * @brief 按 VrA 组和 SwA/SwB 选出模式，同时定下腿长。
 *
 * 模式和腿长由同一组拨杆共同决定（压缩腿就是"模式不变、腿长变短"），
 * 所以放在一个函数里一次算完，不拆成两个各判一遍同样的拨杆。
 *
 * ⚠ 只判 DOWN，其余一律当 UP。SwA/SwB 是两档开关读不出 MID，
 * DR16 降级映射还会留下 UNKNOWN，这么写让这两种情况都落到安全的一侧。
 */
static Chassis_Mode_t Remote_Mode_Select(const Remote_t *remote)
{
    uint8_t a_down = (remote->sw[REMOTE_SW_A] == REMOTE_SWITCH_DOWN) ? 1U : 0U;
    uint8_t b_down = (remote->sw[REMOTE_SW_B] == REMOTE_SWITCH_DOWN) ? 1U : 0U;

    Chassis.goal.L0 = APP_RC_LEG_M;

    if (Chassis.knob_group == 0U)
    {
        if ((a_down != 0U) && (b_down != 0U))
        {
            return CHASSIS_MAP_LO_A_DN_B_DN;
        }
        if (a_down != 0U)
        {
            return CHASSIS_MAP_LO_A_DN_B_UP;
        }
        if (b_down != 0U)
        {
            return CHASSIS_MAP_LO_A_UP_B_DN;
        }
        return CHASSIS_MAP_LO_A_UP_B_UP;
    }

    if (b_down != 0U)
    {
        return CHASSIS_MAP_HI_B_DN;
    }
    if (a_down != 0U)
    {
        Chassis.goal.L0 = APP_RC_LEG_CROUCH;
        return CHASSIS_MAP_HI_A_DN;
    }
    return CHASSIS_MAP_HI_A_UP;
}

/** @brief 将通用遥控输入转换为底盘物理运动目标。 */
static void Remote_Goal_Update(const Remote_t *remote)
{
    float yaw_axis = remote->rightStick.x;

    if (Chassis.mode == CHASSIS_MODE_TOP)
    {
        Chassis.goal.d_s =
            remote->leftStick.y *
            Chassis_Config.top.max_d_s;
        /*
         * 负号把摇杆的"右为正"翻成业务坐标的"Y 左为正"（见 chassis_config.c
         * 开头的坐标约定）。少这个负号时小陀螺平移的左右是反的：投影公式
         * sin 项按左正推导，而摇杆的 leftStick.x 是右正。
         */
        Chassis.goal.d_y =
            -remote->leftStick.x *
            Chassis_Config.top.max_d_s;
        /* 小陀螺按配置的固定转速自转，右摇杆不参与调速。 */
        Chassis.goal.d_fai = Chassis_Config.top.spin_d_fai;
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
    Remote_Switch_t enable_sw = remote->sw[REMOTE_SW_C];

    Chassis.remote_online_flag = (remote->online != 0U) ? 1U : 0U;
    /*
     * ⚠ SwC【上】是急停，与旧的 DR16 右拨杆"下=急停"方向相反。
     * FS-i6X 要求所有拨杆位于 UP 才允许开遥控电源，UP 因此是唯一保证的上电
     * 初始位置，必须映射为最强的封锁。UNKNOWN 一并算急停：通道值残留 0 时
     * IBUS_ConvertSwitch 给出的是 UP 一侧，退化方向也是安全的。
     */
    Chassis.remote_stop_flag =
        ((enable_sw != REMOTE_SWITCH_MID) && (enable_sw != REMOTE_SWITCH_DOWN))
            ? 1U : 0U;

    if ((Chassis.remote_online_flag == 0U) || (Chassis.remote_stop_flag != 0U))
    {
        Chassis.enable_flag = 0U;
        Remote_Hold();
        return;
    }

    Chassis.enable_flag = 1U;

    if (enable_sw == REMOTE_SWITCH_MID)
    {
        /*
         * 只动云台：底盘卸力待命，电机全软可搬运摆姿态。
         * ZERO_FORCE 有最高优先级且会清 fault，所以这一档同时是"万能出口"——
         * 任何卡死状态拨到 SwC 中都能把车拉回零力矩。
         */
        Chassis.mode = CHASSIS_MODE_ZERO_FORCE;
        Remote_Hold();
        return;
    }

    Remote_Group_Update(remote);
    Chassis.mode = Remote_Mode_Select(remote);
    if (Chassis.mode == CHASSIS_MODE_JUMP)
    {
        /*
         * ⚠ 跳跃状态机【尚未实现】，先回落到 FOLLOW，不能让 mode 停在 JUMP——
         * Chassis_State_Update 认不出它，会走到未定义的分支。
         * 实现跳跃时连带要补的还有"跳完自动回 STANDING"的锁存：跳跃是一次性
         * 动作，做完时 SwB 还停在 DOWN，不锁存就会立刻再跳一次。那是整套映射
         * 里唯一需要破例做边沿的地方，设计见 Codex文档/遥控键位与模式分配.md。
         */
        Chassis.mode = CHASSIS_MODE_FOLLOW;
    }
    Remote_Goal_Update(remote);
}
