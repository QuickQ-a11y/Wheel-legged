#include "chassis_remote.h"

#include "app_config.h"

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
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.remote_stop_flag == 1U);
    assert(Chassis.enable_flag == 0U);

    /* 通道值残留 0 会解出 UNKNOWN，必须一并退化成急停。 */
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_UNKNOWN;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.remote_stop_flag == 1U);
    assert(Chassis.enable_flag == 0U);

    /* 中档：只动云台，底盘卸力待命，运动目标全部回中。 */
    Chassis.goal.d_s = 1.0f;
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_MID;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.remote_stop_flag == 0U);
    assert(Chassis.enable_flag == 1U);
    assert(Chassis.mode == CHASSIS_MODE_ZERO_FORCE);
    assert(Chassis.goal.d_s == 0.0f);

    /* 下档：交给 VrA + SwA/SwB 选模式。 */
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 0U);
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
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);

    /*
     * ⚠ 模式不能在这里被压成 ZERO_FORCE：那个模式在 Chassis_State_Update 里
     * 会把 Chassis.fault 整个清零，连 CHASSIS_FAULT_REMOTE 一起抹掉，
     * 于是"遥控丢了"在 Watch 上就看不出来了。封输出是 fault 路径的事。
     */
    remote.online = 0U;
    Chassis.goal.d_s = 1.0f;
    Chassis_Remote_Update(&remote, 0U);
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

    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_UP;
    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_STEP);

    /* 两个都打下去：小陀螺和台阶互斥，无从选择，回落到 FOLLOW。 */
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 0U);
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

    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.knob_group == 1U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    /* SwA 下 = 压缩腿：模式仍是 FOLLOW，只有腿长换成 CROUCH。 */
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_CROUCH);

    assert(Chassis.goal.jump_flag == 0U);

    /*
     * SwB 下 = 跳跃，无论 SwA 在哪（跳跃本身就含压腿蓄力，不冲突）。
     * 跳跃不是一个模式：mode 仍是 FOLLOW，只多一个 jump_flag 电平，
     * 由控制层取上升沿。腿长也要回到行驶值，不能留在 CROUCH。
     */
    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert(Chassis.goal.jump_flag == 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_UP;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert(Chassis.goal.jump_flag == 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_UP;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.goal.jump_flag == 0U);
}

/** @brief VrA 切组带滞回：中间带里保持当前组，不随噪声横跳。 */
static void testKnobHysteresis(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN; /* 低组=TOP，高组=压缩腿 */

    remote.knobA = -1.0f;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.knob_group == 0U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);

    /* 从低组往上扫，没越过 +HYST 之前一直留在低组。 */
    remote.knobA = 0.0f;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.knob_group == 0U);
    remote.knobA = APP_RC_KNOB_HYST;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.knob_group == 0U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);

    /* 越过才换组。 */
    remote.knobA = APP_RC_KNOB_HYST + 0.01f;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.knob_group == 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_CROUCH);

    /* 回中间带，仍然留在高组——这就是滞回要的效果。 */
    remote.knobA = 0.0f;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.knob_group == 1U);
    remote.knobA = -APP_RC_KNOB_HYST;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.knob_group == 1U);

    remote.knobA = -APP_RC_KNOB_HYST - 0.01f;
    Chassis_Remote_Update(&remote, 0U);
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
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_MID;
    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_MID;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
}

/** @brief 摇杆到运动目标的映射，含小陀螺的二维平移和固定自转。 */
static void testStickMapping(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    remote.leftStick.y = 1.0f;
    remote.rightStick.x = 1.0f;
    Chassis_Remote_Update(&remote, 0U);
    assert_near(Chassis.goal.d_s, APP_RC_MAX_VEL);
    /* 右摇杆给的是偏航角速度，航向目标由控制层积分。 */
    assert_near(Chassis.goal.d_fai, -APP_RC_MAX_YAW);
    assert(Chassis.goal.d_y == 0.0f);

    remote.leftStick.y = 0.0f;
    remote.rightStick.x = 0.0f;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.goal.d_fai == 0.0f);

    /* 小陀螺：左摇杆二维平移，右摇杆不参与调速，转速恒为配置值。 */
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    remote.leftStick.y = 1.0f;
    remote.leftStick.x = -0.5f;
    remote.rightStick.x = 1.0f;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);
    assert_near(Chassis.goal.d_s, Chassis_Config.top.max_d_s);
    /* d_y 是左正、摇杆右正，所以推杆左半程给出正的 d_y。 */
    assert_near(Chassis.goal.d_y, 0.5f * Chassis_Config.top.max_d_s);
    assert_near(Chassis.goal.d_fai, Chassis_Config.top.spin_d_fai);

    remote.rightStick.x = -1.0f;
    Chassis_Remote_Update(&remote, 0U);
    assert_near(Chassis.goal.d_fai, Chassis_Config.top.spin_d_fai);

    remote.rightStick.x = 0.0f;
    Chassis_Remote_Update(&remote, 0U);
    assert_near(Chassis.goal.d_fai, Chassis_Config.top.spin_d_fai);
}

/** @brief 模式纯电平跟随：任何内部 state 下拨一下就改，不存在模式卡死。 */
static void testLevelFollowing(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    remote.sw[REMOTE_SW_B] = REMOTE_SWITCH_DOWN;
    Chassis.state = CHASSIS_FALLEN;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_STEP);
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_STEP);

    /* 卡在零力状态时拨到 SwC 中必须立刻生效——这是"万能出口"。 */
    Chassis.state = CHASSIS_ZERO_FORCE;
    Chassis.last_mode = CHASSIS_MODE_STEP;
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_MID;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_ZERO_FORCE);

    /* 拨回运行档，模式立刻重新跟随 SwA/SwB，不需要额外的建立许可步骤。 */
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_DOWN;
    Chassis.state = CHASSIS_STANDING;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.enable_flag == 1U);
    assert(Chassis.mode == CHASSIS_MODE_STEP);
}


/**
 * @brief 本地 DT7/DR16 时，滚轮选三档腿长。
 *
 * ⚠ 三个常量的名字是【历史口径】，不是按长度排的：M 才是三档里最短的，
 * S 反而更长。所以这里断言的是"滚轮推到哪一边得到哪个常量"，
 * 不是"推到正端腿更短"——照后者写会被名字骗过去。
 */
static void testLocalDr16DialSelectsLeg(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    /* 滚轮推过正阈值 -> S 档。 */
    remote.knobA = APP_RC_DIAL_THRESHOLD + 0.1f;
    Chassis_Remote_Update(&remote, 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_S);

    /* 推过负阈值 -> L 档。 */
    remote.knobA = -(APP_RC_DIAL_THRESHOLD + 0.1f);
    Chassis_Remote_Update(&remote, 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_L);

    /* 中间带 -> M 档，也是滚轮字段无效（恒 0）时的退化值。 */
    remote.knobA = 0.0f;
    Chassis_Remote_Update(&remote, 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    /* 阈值本身不算过界，两侧都要落回中间带。 */
    remote.knobA = APP_RC_DIAL_THRESHOLD;
    Chassis_Remote_Update(&remote, 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);
    remote.knobA = -APP_RC_DIAL_THRESHOLD;
    Chassis_Remote_Update(&remote, 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);
}

/**
 * @brief 本地 DR16 时滚轮【不再】选模式组，组锁死在低组。
 *
 * 少了这条，"滚轮同时干两件事"的实现也能过上一条用例：
 * 滚轮推到正端既选了 S 档腿长、又把组切成了高组，模式会跟着变。
 */
static void testLocalDr16LocksLowGroup(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    /* 滚轮推到正端（本来会切高组），SwA 打下去在低组应当是小陀螺。 */
    remote.knobA = 1.0f;
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 1U);
    assert(Chassis.knob_group == 0U);
    assert(Chassis.mode == CHASSIS_MAP_LO_A_DN_B_UP);

    /* 滚轮推到负端同理，组仍然是低组。 */
    remote.knobA = -1.0f;
    Chassis_Remote_Update(&remote, 1U);
    assert(Chassis.knob_group == 0U);
    assert(Chassis.mode == CHASSIS_MAP_LO_A_DN_B_UP);
}

/**
 * @brief 板间转发（FS-i6X）时行为一个字不变：VrA 仍然选组，腿长仍然恒为 M。
 *
 * 这条是回归保险——滚轮那套只能活在本地 DR16 那条路上，
 * 漏判 local_dr16 会让 i6X 的模式组选择整个失效。
 */
static void testBoardSourceUnaffectedByKnob(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    remote.knobA = 1.0f; /* 高组 */
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.knob_group == 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    remote.knobA = -1.0f; /* 低组 */
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.knob_group == 0U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);
}

/**
 * @brief 本地 DR16 那条路上，压缩腿档到不了（高组够不着），腿长只由滚轮定。
 */
static void testLocalDr16HasNoCrouch(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    /* 高组 + SwA 下 是 i6X 的压缩腿档；本地 DR16 锁低组，够不到。 */
    remote.knobA = 1.0f;
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 1U);
    assert(fabsf(Chassis.goal.L0 - APP_RC_LEG_CROUCH) > TEST_TOLERANCE);
}

/**
 * @brief 本地 DT7/DR16：左拨杆打到最上请求跳跃，中间是普通站立，最下是小陀螺。
 *
 * ⚠ 这条只属于本地 DR16。i6X 的 SwA 在 UP 是【常态】（上电要求全部拨杆在上），
 * 那条路上 UP 置跳跃的话，一上电站起来就会跳，所以要单独钉住 i6X 不受影响。
 */
static void testLocalDr16LeftUpRequestsJump(void)
{
    Remote_t remote = testBaseRemote();

    memset(&Chassis, 0, sizeof(Chassis));

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_UP;
    Chassis_Remote_Update(&remote, 1U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert(Chassis.goal.jump_flag == 1U);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_MID;
    Chassis_Remote_Update(&remote, 1U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert(Chassis.goal.jump_flag == 0U);

    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 1U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);
    assert(Chassis.goal.jump_flag == 0U);

    /* 急停和卸力待命都必须把请求撤掉，否则恢复运行那一拍就是个现成的上升沿。 */
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_UP;
    Chassis_Remote_Update(&remote, 1U);
    assert(Chassis.goal.jump_flag == 1U);
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_UP;
    Chassis_Remote_Update(&remote, 1U);
    assert(Chassis.goal.jump_flag == 0U);

    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_DOWN;
    Chassis_Remote_Update(&remote, 1U);
    assert(Chassis.goal.jump_flag == 1U);
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_MID;
    Chassis_Remote_Update(&remote, 1U);
    assert(Chassis.goal.jump_flag == 0U);

    /* i6X 那条路上 SwA 在 UP 是常态，绝不能请求跳跃。 */
    remote.sw[REMOTE_SW_C] = REMOTE_SWITCH_DOWN;
    remote.sw[REMOTE_SW_A] = REMOTE_SWITCH_UP;
    remote.knobA = -1.0f;
    Chassis_Remote_Update(&remote, 0U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert(Chassis.goal.jump_flag == 0U);
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
    testLocalDr16DialSelectsLeg();
    testLocalDr16LocksLowGroup();
    testBoardSourceUnaffectedByKnob();
    testLocalDr16HasNoCrouch();
    testLocalDr16LeftUpRequestsJump();
    return 0;
}
