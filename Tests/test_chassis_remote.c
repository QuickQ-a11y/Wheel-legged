#include "chassis_task.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define TEST_TOLERANCE 1.0e-6f

chassis_t chassis;

static void assertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) <= TEST_TOLERANCE);
}

int main(void)
{
    remote_input_t input = {
        .modeRequest = REMOTE_MODE_BENCH,
        .legRequest = REMOTE_LEG_MIDDLE,
    };

    memset(&chassis, 0, sizeof(chassis));
    chassis.imu.yaw_total_rad = 0.30f;
    chassis.motion_command.leg_length_m = APP_RC_LEG_M;

    Chassis_SetRemoteInput(NULL, 1U);
    assert(chassis.remote_online == 0U);
    assert(chassis.remote_control_ready == 0U);
    assert(chassis.enabled == 0U);

    /* 首次上线时没有FOLLOW请求，不允许直接进入板凳模式。 */
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.remote_online == 1U);
    assert(chassis.remote_control_ready == 0U);
    assert(chassis.mode == CHASSIS_MODE_ZERO_FORCE);

    input.modeRequest = REMOTE_MODE_FOLLOW;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.remote_control_ready == 1U);
    assert(chassis.remote_target_valid == 1U);
    assert(chassis.enabled == 1U);
    assert(chassis.mode == CHASSIS_MODE_FOLLOW);
    assertNear(chassis.motion_command.yaw_anchor_rad, 0.30f);

    input.forwardAxis = 1.0f;
    input.yawAxis = 1.0f;
    input.legRequest = REMOTE_LEG_SHORT;
    Chassis_SetRemoteInput(&input, 1U);
    assertNear(chassis.motion_command.forward_speed_mps, APP_RC_MAX_VEL);
    assertNear(chassis.motion_command.yaw_target_rad,
               0.30f - APP_RC_MAX_YAW);
    assertNear(chassis.motion_command.leg_length_m, APP_RC_LEG_S);

    input.yawAxis = 0.0f;
    input.forwardAxis = 0.0f;
    input.legRequest = REMOTE_LEG_MIDDLE;
    chassis.imu.yaw_total_rad = 0.45f;
    Chassis_SetRemoteInput(&input, 1U);
    assertNear(chassis.motion_command.yaw_anchor_rad, 0.45f);
    assertNear(chassis.motion_command.yaw_target_rad, 0.45f);
    assertNear(chassis.motion_command.leg_length_m, APP_RC_LEG_M);

    input.legRequest = REMOTE_LEG_LONG;
    Chassis_SetRemoteInput(&input, 1U);
    assertNear(chassis.motion_command.leg_length_m, APP_RC_LEG_L);
    input.legRequest = REMOTE_LEG_KEEP;
    Chassis_SetRemoteInput(&input, 1U);
    assertNear(chassis.motion_command.leg_length_m, APP_RC_LEG_L);

    input.modeRequest = REMOTE_MODE_BENCH;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.mode == CHASSIS_MODE_BENCH);

    input.modeRequest = REMOTE_MODE_SELF_SAVE;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.mode == CHASSIS_MODE_SELF_SAVE);
    assert(chassis.remote_self_save_latched == 1U);

    /* 触发当周期仍是STANDING，不得被误判为自救已经完成。 */
    chassis.state = CHASSIS_STANDING;
    chassis.last_mode = CHASSIS_MODE_BENCH;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.mode == CHASSIS_MODE_SELF_SAVE);

    chassis.last_mode = CHASSIS_MODE_SELF_SAVE;
    chassis.state = CHASSIS_FALLEN;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.mode == CHASSIS_MODE_SELF_SAVE);

    chassis.state = CHASSIS_STANDING;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.mode == CHASSIS_MODE_FOLLOW);
    assert(chassis.remote_self_save_latched == 1U);

    /* 持续SELF_SAVE不会重复触发，收到FOLLOW才解除触发锁。 */
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.mode == CHASSIS_MODE_FOLLOW);
    input.modeRequest = REMOTE_MODE_FOLLOW;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.remote_self_save_latched == 0U);

    input.modeRequest = REMOTE_MODE_BENCH;
    Chassis_SetRemoteInput(&input, 1U);
    input.modeRequest = REMOTE_MODE_SELF_SAVE;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.mode == CHASSIS_MODE_SELF_SAVE);

    /* 急停只关闭输出许可，保留SELF_SAVE模式供中间量继续计算。 */
    input.stop = 1U;
    chassis.motion_command.forward_speed_mps = APP_RC_MAX_VEL;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.remote_stop == 1U);
    assert(chassis.remote_control_ready == 0U);
    assert(chassis.enabled == 0U);
    assert(chassis.mode == CHASSIS_MODE_SELF_SAVE);
    assert(chassis.motion_command.forward_speed_mps == 0.0f);

    input.stop = 0U;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.remote_control_ready == 0U);
    assert(chassis.enabled == 0U);
    input.modeRequest = REMOTE_MODE_FOLLOW;
    chassis.state = CHASSIS_FALLEN;
    Chassis_SetRemoteInput(&input, 1U);
    assert(chassis.remote_control_ready == 1U);
    assert(chassis.enabled == 1U);
    assert(chassis.mode == CHASSIS_MODE_SELF_SAVE);

    Chassis_SetRemoteInput(&input, 0U);
    assert(chassis.remote_online == 0U);
    assert(chassis.remote_control_ready == 0U);
    assert(chassis.enabled == 0U);
    assert(chassis.mode == CHASSIS_MODE_SELF_SAVE);
    return 0;
}
