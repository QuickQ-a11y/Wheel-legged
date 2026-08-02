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
    remote_input_t input = {
        .modeRequest = REMOTE_MODE_BENCH,
        .legRequest = REMOTE_LEG_MIDDLE,
    };

    memset(&Chassis, 0, sizeof(Chassis));
    Chassis_Remote_Init();
    Chassis.imu.yaw_total = 0.30f;
    Chassis.goal.L0 = APP_RC_LEG_M;

    Chassis_Remote_Update(NULL, 1U);
    assert(Chassis.remote_online_flag == 0U);
    assert(Chassis.remote_ready_flag == 0U);
    assert(Chassis.enable_flag == 0U);

    /* 首次上线时没有FOLLOW请求，不允许直接进入板凳模式。 */
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.remote_online_flag == 1U);
    assert(Chassis.remote_ready_flag == 0U);
    assert(Chassis.mode == CHASSIS_MODE_ZERO_FORCE);

    input.modeRequest = REMOTE_MODE_FOLLOW;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.remote_ready_flag == 1U);
    assert(Chassis.remote_target_flag == 1U);
    assert(Chassis.enable_flag == 1U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert_near(Chassis.goal.fai_anchor, 0.30f);

    input.forwardAxis = 1.0f;
    input.yawAxis = 1.0f;
    input.legRequest = REMOTE_LEG_SHORT;
    Chassis_Remote_Update(&input, 1U);
    assert_near(Chassis.goal.d_s, APP_RC_MAX_VEL);
    assert_near(Chassis.goal.fai,
               0.30f - APP_RC_MAX_YAW);
    assert_near(Chassis.goal.L0, APP_RC_LEG_S);

    input.yawAxis = 0.0f;
    input.forwardAxis = 0.0f;
    input.legRequest = REMOTE_LEG_MIDDLE;
    Chassis.imu.yaw_total = 0.45f;
    Chassis_Remote_Update(&input, 1U);
    assert_near(Chassis.goal.fai_anchor, 0.45f);
    assert_near(Chassis.goal.fai, 0.45f);
    assert_near(Chassis.goal.L0, APP_RC_LEG_M);

    input.legRequest = REMOTE_LEG_LONG;
    Chassis_Remote_Update(&input, 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_L);
    input.legRequest = REMOTE_LEG_KEEP;
    Chassis_Remote_Update(&input, 1U);
    assert_near(Chassis.goal.L0, APP_RC_LEG_L);

    input.modeRequest = REMOTE_MODE_TOP;
    input.forwardAxis = 1.0f;
    input.lateralAxis = -0.5f;
    input.yawAxis = 1.0f;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.mode == CHASSIS_MODE_TOP);
    assert_near(Chassis.goal.d_s,
               Chassis_Config.top.max_d_s);
    assert_near(Chassis.goal.d_y,
               -0.5f * Chassis_Config.top.max_d_s);
    assert_near(Chassis.goal.d_fai,
               -Chassis_Config.top.max_d_fai);

    input.modeRequest = REMOTE_MODE_STEP;
    input.forwardAxis = 0.5f;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.mode == CHASSIS_MODE_STEP);
    assert_near(Chassis.goal.d_s,
               0.5f * APP_RC_MAX_VEL);
    assert(Chassis.goal.d_y == 0.0f);
    assert(Chassis.goal.d_fai == 0.0f);

    input.modeRequest = REMOTE_MODE_BENCH;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.mode == CHASSIS_MODE_BENCH);

    input.modeRequest = REMOTE_MODE_SELF_SAVE;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);
    assert(Chassis.recovery_latch_flag == 1U);

    /* 触发当周期仍是STANDING，不得被误判为自救已经完成。 */
    Chassis.state = CHASSIS_STANDING;
    Chassis.last_mode = CHASSIS_MODE_BENCH;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);

    Chassis.last_mode = CHASSIS_MODE_SELF_SAVE;
    Chassis.state = CHASSIS_FALLEN;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);

    Chassis.state = CHASSIS_STANDING;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    assert(Chassis.recovery_latch_flag == 1U);

    /* 持续SELF_SAVE不会重复触发，收到FOLLOW才解除触发锁。 */
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.mode == CHASSIS_MODE_FOLLOW);
    input.modeRequest = REMOTE_MODE_FOLLOW;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.recovery_latch_flag == 0U);

    input.modeRequest = REMOTE_MODE_BENCH;
    Chassis_Remote_Update(&input, 1U);
    input.modeRequest = REMOTE_MODE_SELF_SAVE;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);

    /* 急停只关闭输出许可，保留SELF_SAVE模式供中间量继续计算。 */
    input.stop = 1U;
    Chassis.goal.d_s = APP_RC_MAX_VEL;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.remote_stop_flag == 1U);
    assert(Chassis.remote_ready_flag == 0U);
    assert(Chassis.enable_flag == 0U);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);
    assert(Chassis.goal.d_s == 0.0f);

    input.stop = 0U;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.remote_ready_flag == 0U);
    assert(Chassis.enable_flag == 0U);
    input.modeRequest = REMOTE_MODE_FOLLOW;
    Chassis.state = CHASSIS_FALLEN;
    Chassis_Remote_Update(&input, 1U);
    assert(Chassis.remote_ready_flag == 1U);
    assert(Chassis.enable_flag == 1U);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);

    Chassis_Remote_Update(&input, 0U);
    assert(Chassis.remote_online_flag == 0U);
    assert(Chassis.remote_ready_flag == 0U);
    assert(Chassis.enable_flag == 0U);
    assert(Chassis.mode == CHASSIS_MODE_SELF_SAVE);
    return 0;
}
