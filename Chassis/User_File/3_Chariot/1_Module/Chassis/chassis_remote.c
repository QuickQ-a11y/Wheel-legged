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
    /* 不撤掉的话，恢复运行那一拍就是一个现成的上升沿。 */
    Chassis.goal.jump_flag = 0U;
}

/**
 * @brief 滚轮选三档腿长，只用在【本地 DT7/DR16】那条路上。
 *
 * ⚠⚠ 三个常量的名字不是按长度排的：M 是【原拨轮中位档】、也是三档里最短的，
 * S(0.28) 反而比 M(0.18) 长。照"S 一定最短"去读会把方向搞反。
 *
 * 滚轮字段无效时 DR16_MakeRemote 不写 knobA，留在 0，正好落进中间带拿到 M
 * ——那是三档里最短的一档，退化方向是安全的。
 */
static float Remote_Leg_Select(const Remote_t *remote)
{
    if (remote->knobA > APP_RC_DIAL_THRESHOLD)
    {
        return APP_RC_LEG_S;
    }
    if (remote->knobA < -APP_RC_DIAL_THRESHOLD)
    {
        return APP_RC_LEG_L;
    }
    return APP_RC_LEG_M;
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
 * @brief 按 VrA 组和 SwA/SwB 选出模式，同时定下腿长和跳跃请求。
 *
 * 模式、腿长和跳跃请求由同一组拨杆共同决定（压缩腿就是"模式不变、腿长变短"，
 * 跳跃就是"模式不变、多一个请求"），所以放在一个函数里一次算完，
 * 不拆成几个各判一遍同样的拨杆。
 *
 * ⚠ 选模式只判 DOWN，其余一律当 UP。SwA/SwB 是两档开关读不出 MID，
 * DR16 降级映射还会留下 UNKNOWN，这么写让这两种情况都落到安全的一侧。
 */
static Chassis_Mode_t Remote_Mode_Select(const Remote_t *remote,
                                        uint8_t local_dr16)
{
    uint8_t a_down = (remote->sw[REMOTE_SW_A] == REMOTE_SWITCH_DOWN) ? 1U : 0U;
    uint8_t b_down = (remote->sw[REMOTE_SW_B] == REMOTE_SWITCH_DOWN) ? 1U : 0U;

    /*
     * 本地 DT7/DR16 时腿长由滚轮定；板间转发的 FS-i6X 没有多余旋钮，
     * 恒为 M，只有下面高组 SwA 打下去的压缩腿档才换成 CROUCH。
     */
    Chassis.goal.L0 =
        (local_dr16 != 0U) ? Remote_Leg_Select(remote) : APP_RC_LEG_M;
    /*
     * 本地 DT7/DR16：左拨杆（映射成 SwA）打到最上请求跳跃，模式照旧是 FOLLOW。
     * ⚠ 只认本地 DR16。i6X 的 SwA 在 UP 是常态（上电要求全部拨杆在上），
     * 那条路上这么判，一站起来就会跳。i6X 的跳跃在下面高组的 SwB。
     */
    Chassis.goal.jump_flag =
        ((local_dr16 != 0U) &&
         (remote->sw[REMOTE_SW_A] == REMOTE_SWITCH_UP)) ? 1U : 0U;

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
        Chassis.goal.jump_flag = 1U;
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
void Chassis_Remote_Update(const Remote_t *remote, uint8_t local_dr16)
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

    /*
     * ⚠ 本地 DT7/DR16 上滚轮已经被腿长占了，没有旋钮可以选组，所以锁低组。
     * 低组给的是 FOLLOW / TOP（SwB 在 DR16 上恒为 UNKNOWN，按 UP 处理），
     * 丢掉的高组只有压缩腿和跳跃——压缩腿本来就是一个固定腿长，滚轮自己够得到；
     * 跳跃在本地 DR16 上改由左拨杆打到最上触发，见 Remote_Mode_Select()。
     */
    if (local_dr16 != 0U)
    {
        Chassis.knob_group = 0U;
    }
    else
    {
        Remote_Group_Update(remote);
    }
    Chassis.mode = Remote_Mode_Select(remote, local_dr16);
    Remote_Goal_Update(remote);
}
