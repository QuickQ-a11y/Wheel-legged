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

int main(void)
{
    Remote_t remote = {
        .online = 1U,
        .rightSwitch = REMOTE_SWITCH_UP,
        .modeRequest = REMOTE_MODE_ZERO_FORCE,
        .legRequest = REMOTE_LEG_MIDDLE,
    };
    Remote_t offline_remote = {0};

    memset(&Chassis, 0, sizeof(Chassis));
    Chassis.imu.yaw_total = 0.30f;
    Chassis.goal.L0 = APP_RC_LEG_M;

    /* 遥控离线时不建立控制许可。 */
    Chassis_Remote_Update(&offline_remote);
    assert(Chassis.remote_online_flag == 0U);
    assert(Chassis.enable_flag == 0U);

    /* 模式直接跟随拨杆档位，不需要先送FOLLOW建立许可。 */
    Chassis_Remote_Update(&remote);
    assert(Chassis.remote_online_flag == 1U);
    assert(Chassis.enable_flag == 1U);
    assert(Chassis.mode == CHASSIS_MODE_ZERO_FORCE);

    remote.modeRequest = REMOTE_MODE_FOLLOW;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert(Chassis.goal.d_fai == 0.0f);

    remote.leftStick.y = 1.0f;
    remote.rightStick.x = 1.0f;
    remote.legRequest = REMOTE_LEG_SHORT;
    Chassis_Remote_Update(&remote);
    assert_near(Chassis.goal.d_s, APP_RC_MAX_VEL);
    /* 右摇杆给的是偏航角速度，航向目标由控制层积分。 */
    assert_near(Chassis.goal.d_fai, -APP_RC_MAX_YAW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_S);

    remote.rightStick.x = 0.0f;
    remote.leftStick.y = 0.0f;
    remote.legRequest = REMOTE_LEG_MIDDLE;
    Chassis.imu.yaw_total = 0.45f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.goal.d_fai == 0.0f);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    remote.legRequest = REMOTE_LEG_LONG;
    Chassis_Remote_Update(&remote);
    assert_near(Chassis.goal.L0, APP_RC_LEG_L);
    remote.legRequest = REMOTE_LEG_KEEP;
    Chassis_Remote_Update(&remote);
    assert_near(Chassis.goal.L0, APP_RC_LEG_L);

    /*
     * 小陀螺按配置的固定转速自转：右摇杆无论推到哪里，goal.d_fai 都等于
     * spin_d_fai。左摇杆的二维平移仍然生效。
     */
    remote.modeRequest = REMOTE_MODE_TOP;
    remote.leftStick.y = 1.0f;
    remote.leftStick.x = -0.5f;
    remote.rightStick.x = 1.0f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_TOP);
    assert_near(Chassis.goal.d_s,
               Chassis_Config.top.max_d_s);
    assert_near(Chassis.goal.d_y,
               -0.5f * Chassis_Config.top.max_d_s);
    assert_near(Chassis.goal.d_fai,
               Chassis_Config.top.spin_d_fai);

    /* 右摇杆推到反向满杆，转速仍然不变。 */
    remote.rightStick.x = -1.0f;
    Chassis_Remote_Update(&remote);
    assert_near(Chassis.goal.d_fai,
               Chassis_Config.top.spin_d_fai);

    /* 右摇杆回中，转速依旧是配置值，不会掉到 0。 */
    remote.rightStick.x = 0.0f;
    Chassis_Remote_Update(&remote);
    assert_near(Chassis.goal.d_fai,
               Chassis_Config.top.spin_d_fai);

    remote.modeRequest = REMOTE_MODE_STEP;
    remote.leftStick.y = 0.5f;
    remote.rightStick.x = 0.0f;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_STEP);
    assert_near(Chassis.goal.d_s,
               0.5f * APP_RC_MAX_VEL);
    assert(Chassis.goal.d_y == 0.0f);
    assert(Chassis.goal.d_fai == 0.0f);

    /*
     * 自救是电平触发：拨杆停在自救档就一直是SELF_SAVE，拨走立刻变成
     * 别的模式。任何state下都能改，因此不存在模式和状态互锁。
     */
    remote.modeRequest = REMOTE_MODE_SELF_SAVE;
    Chassis.state = CHASSIS_FALLEN;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);

    /* 恢复超时落到ZERO_FORCE，此时拨到零力矩档必须立刻生效。 */
    Chassis.state = CHASSIS_ZERO_FORCE;
    Chassis.last_mode = CHASSIS_MODE_SELF_SAVE;
    remote.modeRequest = REMOTE_MODE_ZERO_FORCE;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_ZERO_FORCE);

    /* 自救成功回到STANDING后拨回FOLLOW同样立刻生效。 */
    remote.modeRequest = REMOTE_MODE_SELF_SAVE;
    Chassis_Remote_Update(&remote);
    Chassis.state = CHASSIS_STANDING;
    remote.modeRequest = REMOTE_MODE_FOLLOW;
    Chassis_Remote_Update(&remote);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);

    /* 急停只关闭输出许可并回中目标，模式保持不变。 */
    remote.modeRequest = REMOTE_MODE_SELF_SAVE;
    Chassis_Remote_Update(&remote);
    remote.rightSwitch = REMOTE_SWITCH_DOWN;
    Chassis.goal.d_s = APP_RC_MAX_VEL;
    Chassis_Remote_Update(&remote);
    assert(Chassis.remote_stop_flag == 1U);
    assert(Chassis.enable_flag == 0U);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);
    assert(Chassis.goal.d_s == 0.0f);

    /* 解除急停后模式立刻重新跟随拨杆，不需要额外的建立许可步骤。 */
    remote.rightSwitch = REMOTE_SWITCH_UP;
    remote.modeRequest = REMOTE_MODE_FOLLOW;
    Chassis_Remote_Update(&remote);
    assert(Chassis.enable_flag == 1U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);

    remote.online = 0U;
    Chassis_Remote_Update(&remote);
    assert(Chassis.remote_online_flag == 0U);
    assert(Chassis.enable_flag == 0U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    return 0;
}
