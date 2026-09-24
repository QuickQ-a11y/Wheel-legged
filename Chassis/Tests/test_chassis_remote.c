#include "chassis_remote.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define TEST_TOLERANCE 1.0e-6f

Chassis_t Chassis;

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) <= TEST_TOLERANCE);
}

/** @brief 造一份在线、SwC 在下（运行档）、其余拨杆在上的基准输入。 */
static Remote_t testBaseRemote(void)
{
    Remote_t remote = {0};

    remote.online = 1U;
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_UP;
    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_UP;
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_DOWN;
    remote.sw[REMOTE_SW_D] = REMOTE_SWITCH_UP;
    remote.knobA = -1.0f; /* 低组 */
    return remote;
}

/** @brief SwC 决定使能级：上和未知都急停，中是卸力待命，下才运行。 */
static void testEnableLevel(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    /*
     * ⚠ 方向性判据：SwC【上】必须是急停。FS-i6X 要求所有拨杆在 UP 才允许
     * 开遥控电源，所以 UP 是唯一保证的上电初始位置。这条判反了会让操作者
     * 以为在打急停、实际在选运行——把它钉死在这里。
     */
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_UP;
    Chassis_Remote_Update(&remote);
    assert(Chassis.remote_stop_flag == 1U);
    assert(Chassis.enable_flag == 0U);

    /* 通道值残留 0 会解出 UNKNOWN，必须一并退化成急停。 */
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_UNKNOWN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.remote_stop_flag == 1U);
    assert(Chassis.enable_flag == 0U);

    /* 中档：只动云台，底盘卸力待命，运动目标全部回中。 */
    Chassis.goal.d_s = 1.0f;
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_MID;
    Chassis_Remote_Update(&remote);
    assert(Chassis.remote_stop_flag == 0U);
    assert(Chassis.enable_flag == 1U);
    assert(Chassis.mode == CHASSIS_MODE_ZERO_FORCE);
    assert(Chassis.goal.d_s == 0.0f);

    /* 下档：交给 VrA + SwA/SwB 选模式。 */
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.remote_stop_flag == 0U);
    assert(Chassis.enable_flag == 1U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
}

/** @brief 离线只关许可，模式保持不变（急停同理，两者都走 fault 路径封输出）。 */
static void testOffline(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN; /* 低组 SwA 下 = 小陀螺 */
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_TOP);

    /*
     * ⚠ 模式不能在这里被压成 ZERO_FORCE：那个模式在 Chassis_State_Update 里
     * 会把 Chassis.fault 整个清零，连 CHASSIS_FAULT_REMOTE 一起抹掉，
     * 于是"遥控丢了"在 Watch 上就看不出来了。封输出是 fault 路径的事。
     */
    remote.online = 0U;
    Chassis.goal.d_s = 1.0f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.remote_online_flag == 0U);
    assert(Chassis.enable_flag == 0U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);
    assert(Chassis.goal.d_s == 0.0f);
}

/** @brief VrA 低组的四种 SwA/SwB 组合。 */
static void testLowGroup(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_TOP);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_UP;
    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_STEP);

    /* 两个都打下去：小陀螺和台阶互斥，无从选择，回落到 FOLLOW。 */
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);

    /* 低组腿长恒为行驶腿长，不随 SwA/SwB 变。 */
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);
}

/** @brief VrA 高组：SwB 盖过 SwA；压缩腿只改腿长不改模式。 */
static void testHighGroup(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));
    remote.knobA = 1.0f;

    Chassis_Remote_Update(&remote);
    assert(Chassis.knob_group == 1U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    /* SwA 下 = 压缩腿：模式仍是 FOLLOW，只有腿长换成 CROUCH。 */
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_CROUCH);

    /*
     * SwB 下 = 跳跃，无论 SwA 在哪（跳跃本身就含压腿蓄力，不冲突）。
     * ⚠ 跳跃状态机未实现，映射层必须回落到 FOLLOW——mode 停在 JUMP 的话
     * Chassis_State_Update 认不出它。腿长也要回到行驶值，不能留在 CROUCH。
     */
    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_UP;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);
}

/** @brief VrA 切组带滞回：中间带里保持当前组，不随噪声横跳。 */
static void testKnobHysteresis(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN; /* 低组=TOP，高组=压缩腿 */

    remote.knobA = -1.0f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.knob_group == 0U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);

    /* 从低组往上扫，没越过 +HYST 之前一直留在低组。 */
    remote.knobA = 0.0f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.knob_group == 0U);
    remote.knobA = APP_RC_KNOB_HYST;
    Chassis_Remote_Update(&remote);
    assert(Chassis.knob_group == 0U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);

    /* 越过才换组。 */
    remote.knobA = APP_RC_KNOB_HYST + 0.01f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.knob_group == 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_CROUCH);

    /* 回中间带，仍然留在高组——这就是滞回要的效果。 */
    remote.knobA = 0.0f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.knob_group == 1U);
    remote.knobA = -APP_RC_KNOB_HYST;
    Chassis_Remote_Update(&remote);
    assert(Chassis.knob_group == 1U);

    remote.knobA = -APP_RC_KNOB_HYST - 0.01f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.knob_group == 0U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);
}

/**
 * @brief SwA/SwB 只判 DOWN，MID 和 UNKNOWN 一律当 UP。
 *
 * 这两路挂的是两档开关读不出 MID，而 DR16 降级映射会留下 UNKNOWN。
 * 两种情况都必须落到不激活的一侧。
 */
static void testTwoPositionDegradation(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_UNKNOWN;
    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_UNKNOWN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_MID;
    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_MID;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
}

/** @brief 摇杆到运动目标的映射，含小陀螺的二维平移和固定自转。 */
static void testStickMapping(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    remote.leftStick.y = 1.0f;
    remote.rightStick.x = 1.0f;
    Chassis_Remote_Update(&remote);
    assert_near(Chassis.goal.d_s, APP_RC_MAX_VEL);
    /* 右摇杆给的是偏航角速度，航向目标由控制层积分。 */
    assert_near(Chassis.goal.d_fai, -APP_RC_MAX_YAW);
    assert(Chassis.goal.d_y == 0.0f);

    remote.leftStick.y = 0.0f;
    remote.rightStick.x = 0.0f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.goal.d_fai == 0.0f);

    /* 小陀螺：左摇杆二维平移，右摇杆不参与调速，转速恒为配置值。 */
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    remote.leftStick.y = 1.0f;
    remote.leftStick.x = -0.5f;
    remote.rightStick.x = 1.0f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_TOP);
    assert_near(Chassis.goal.d_s, Chassis_Config.top.max_d_s);
    /* d_y 是左正、摇杆右正，所以推杆左半程给出正的 d_y。 */
    assert_near(Chassis.goal.d_y, 0.5f * Chassis_Config.top.max_d_s);
    assert_near(Chassis.goal.d_fai, Chassis_Config.top.spin_d_fai);

    remote.rightStick.x = -1.0f;
    Chassis_Remote_Update(&remote);
    assert_near(Chassis.goal.d_fai, Chassis_Config.top.spin_d_fai);

    remote.rightStick.x = 0.0f;
    Chassis_Remote_Update(&remote);
    assert_near(Chassis.goal.d_fai, Chassis_Config.top.spin_d_fai);
}

/** @brief 模式纯电平跟随：任何内部 state 下拨一下就改，不存在模式卡死。 */
static void testLevelFollowing(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_DOWN;
    Chassis.state = CHASSIS_FALLEN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_STEP);
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_STEP);

    /* 卡在零力状态时拨到 SwC 中必须立刻生效——这是"万能出口"。 */
    Chassis.state = CHASSIS_ZERO_FORCE;
    Chassis.last_mode = CHASSIS_MODE_STEP;
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_MID;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_ZERO_FORCE);

    /* 拨回运行档，模式立刻重新跟随 SwA/SwB，不需要额外的建立许可步骤。 */
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_DOWN;
    Chassis.state = CHASSIS_STANDING;
    Chassis_Remote_Update(&remote);
    assert(Chassis.enable_flag == 1U);
    assert(Chassis.mode == CHASSIS_MODE_STEP);
}

int main(void)
{
    testEnableLevel();
    testOffline();
    testLowGroup();
    testHighGroup();
    testKnobHysteresis();
    testTwoPositionDegradation();
    testStickMapping();
    testLevelFollowing();
    return 0;
}
