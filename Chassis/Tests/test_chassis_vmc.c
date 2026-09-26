#include "chassis_vmc.h"

#include "PID.h"
#include "app_config.h"

#include <assert.h>
#include <math.h>

#define TEST_TOLERANCE 2.0e-5f
#define TEST_DERIVATIVE_TOLERANCE 2.0e-3f

static float motor_position(const Chassis_Leg_Config_t *config,
                            Chassis_Joint_t joint,
                            float phi)
{
    return (phi - config->joint[joint].angle_offset_rad) /
           (config->joint[joint].scale *
            config->joint[joint].ratio);
}

static void calculate_target_pose(const Chassis_Leg_Config_t *config,
                                  float target_L0,
                                  float target_phi0,
                                  Chassis_Leg_t *leg)
{
    Chassis_Leg_t current_leg = {
        .phi1 = 2.7f,
        .phi4 = 0.4f,
    };
    VMC_Joint_Target_t target;
    float phi1_motor_position;
    float phi4_motor_position;

    assert(VMC_Inverse_Calc(config,
                            &current_leg,
                            target_L0,
                            target_phi0,
                            &target) == 1U);
    phi1_motor_position =
        motor_position(config, CHASSIS_JOINT_PHI1, target.phi1);
    phi4_motor_position =
        motor_position(config, CHASSIS_JOINT_PHI4, target.phi4);
    VMC_State_Calc(config,
                   phi1_motor_position,
                   phi4_motor_position,
                   0.0f,
                   0.0f,
                   leg);
    assert(leg->valid_flag == 1U);
}

static void test_geometry_convention(const Chassis_Leg_Config_t *base_config)
{
    static const float lengths[] = {0.18f, 0.25f, 0.32f};
    static const float angles[] = {1.25f, CHASSIS_HALF_PI, 1.90f};
    Chassis_Leg_Config_t config = *base_config;
    Chassis_Leg_t leg;
    uint32_t length_index;
    uint32_t angle_index;

    calculate_target_pose(&config, 0.25f, CHASSIS_HALF_PI, &leg);
    assert(leg.phi1 > leg.phi4);
    assert(fabsf(leg.phi0 - CHASSIS_HALF_PI) < TEST_TOLERANCE);

    config.geometry.l5 = 0.06f;
    for (length_index = 0U;
         length_index < sizeof(lengths) / sizeof(lengths[0]);
         length_index++)
    {
        for (angle_index = 0U;
             angle_index < sizeof(angles) / sizeof(angles[0]);
             angle_index++)
        {
            calculate_target_pose(&config,
                                  lengths[length_index],
                                  angles[angle_index],
                                  &leg);
            assert(leg.phi1 > leg.phi4);
            assert(fabsf(leg.L0 - lengths[length_index]) < TEST_TOLERANCE);
            assert(fabsf(leg.phi0 - angles[angle_index]) < TEST_TOLERANCE);
        }
    }
}

static void test_jacobian_and_virtual_work(
    const Chassis_Leg_Config_t *config)
{
    const float phi1_motor_position = 0.5f;
    const float phi4_motor_position = -0.8f;
    const float phi1_motor_speed = 0.7f;
    const float phi4_motor_speed = -0.4f;
    const float dt = 1.0e-3f;
    const float F0 = -30.0f;
    const float Tp = 1.2f;
    Chassis_Leg_t leg;
    Chassis_Leg_t previous_leg;
    Chassis_Leg_t next_leg;
    VMC_Torque_t torque;
    float numeric_d_L0;
    float numeric_d_phi0;
    float motor_power;
    float virtual_power;

    VMC_State_Calc(config,
                   phi1_motor_position,
                   phi4_motor_position,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &leg);
    VMC_State_Calc(config,
                   phi1_motor_position - phi1_motor_speed * dt,
                   phi4_motor_position - phi4_motor_speed * dt,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &previous_leg);
    VMC_State_Calc(config,
                   phi1_motor_position + phi1_motor_speed * dt,
                   phi4_motor_position + phi4_motor_speed * dt,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &next_leg);
    assert(leg.valid_flag == 1U);
    assert(previous_leg.valid_flag == 1U);
    assert(next_leg.valid_flag == 1U);

    numeric_d_L0 = (next_leg.L0 - previous_leg.L0) / (2.0f * dt);
    numeric_d_phi0 =
        (next_leg.phi0 - previous_leg.phi0) / (2.0f * dt);
    assert(fabsf(numeric_d_L0 - leg.d_L0) <
           TEST_DERIVATIVE_TOLERANCE);
    assert(fabsf(numeric_d_phi0 - leg.d_phi0) <
           TEST_DERIVATIVE_TOLERANCE);

    assert(VMC_Torque_Calc(config, &leg, F0, Tp, &torque) == 1U);
    motor_power = torque.T1 * phi1_motor_speed +
                  torque.T4 * phi4_motor_speed;
    virtual_power = F0 * leg.d_L0 + Tp * leg.d_phi0;
    assert(fabsf(motor_power - virtual_power) < TEST_TOLERANCE);
}

static void test_downward_phi0_branch(
    const Chassis_Leg_Config_t *config)
{
    const float target_L0 = 0.25f;
    const float target_phi0 = CHASSIS_HALF_PI;
    const float phi1_motor_speed = 0.20f;
    const float phi4_motor_speed = -0.15f;
    const float dt = 1.0e-3f;
    const float F0 = -30.0f;
    const float Tp = 1.2f;
    Chassis_Leg_t current_leg = {
        .phi1 = CHASSIS_PI,
        .phi4 = 0.0f,
    };
    VMC_Joint_Target_t target;
    Chassis_Leg_t leg;
    Chassis_Leg_t previous_leg;
    Chassis_Leg_t next_leg;
    VMC_Torque_t torque;
    float phi1_motor_position;
    float phi4_motor_position;
    float numeric_d_L0;
    float numeric_d_phi0;
    float motor_power;
    float virtual_power;

    assert(VMC_Inverse_Calc(config,
                            &current_leg,
                            target_L0,
                            target_phi0,
                            &target) == 1U);
    phi1_motor_position =
        motor_position(config, CHASSIS_JOINT_PHI1, target.phi1);
    phi4_motor_position =
        motor_position(config, CHASSIS_JOINT_PHI4, target.phi4);

    VMC_State_Calc(config,
                   phi1_motor_position,
                   phi4_motor_position,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &leg);
    VMC_State_Calc(config,
                   phi1_motor_position - phi1_motor_speed * dt,
                   phi4_motor_position - phi4_motor_speed * dt,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &previous_leg);
    VMC_State_Calc(config,
                   phi1_motor_position + phi1_motor_speed * dt,
                   phi4_motor_position + phi4_motor_speed * dt,
                   phi1_motor_speed,
                   phi4_motor_speed,
                   &next_leg);

    assert(leg.valid_flag == 1U);
    assert(previous_leg.valid_flag == 1U);
    assert(next_leg.valid_flag == 1U);
    assert(leg.phi1 > leg.phi4);
    assert(fabsf(leg.L0 - target_L0) < TEST_TOLERANCE);
    assert(fabsf(leg.phi0 - target_phi0) < TEST_TOLERANCE);
    numeric_d_L0 = (next_leg.L0 - previous_leg.L0) / (2.0f * dt);
    numeric_d_phi0 =
        (next_leg.phi0 - previous_leg.phi0) / (2.0f * dt);
    assert(fabsf(numeric_d_L0 - leg.d_L0) <
           TEST_DERIVATIVE_TOLERANCE);
    assert(fabsf(numeric_d_phi0 - leg.d_phi0) <
           TEST_DERIVATIVE_TOLERANCE);

    assert(VMC_Torque_Calc(config, &leg, F0, Tp, &torque) == 1U);
    motor_power = torque.T1 * phi1_motor_speed +
                  torque.T4 * phi4_motor_speed;
    virtual_power = F0 * leg.d_L0 + Tp * leg.d_phi0;
    assert(fabsf(motor_power - virtual_power) < TEST_TOLERANCE);
}


/*
 * 2026-09-26 VMAX 修正：J8009P 寄存器 VMAX = 45，而固件从 2.4 版起一直误用 J4310 的 ±30，
 * 关节速度读数长期是"真实 × 30/45"。改回 ±45 后读数变成真实值，于是凡是对着旧读数
 * 手调过的参数都要折算，否则同一个物理动作会给出不同的输出。
 */
#define VMAX_HISTORY 30.0f /* 2.4 版至 2026-09-26 固件里的错值 */

/** @brief 关节位置串级：外环角度 -> 目标角速度，内环角速度 -> 力矩。与 Joint_Control 同构。 */
static float joint_cascade(const algorithm_pid_config_t *angle_cfg,
                           const algorithm_pid_config_t *speed_cfg,
                           float angle_error,
                           float speed_feedback)
{
    algorithm_pid_state_t angle_state;
    algorithm_pid_state_t speed_state;
    float target_speed = 0.0f;
    float torque = 0.0f;

    Algorithm_PID_Init(&angle_state);
    Algorithm_PID_Init(&speed_state);
    Algorithm_PID_UpdateByFeedbackRate(angle_cfg, &angle_state, angle_error, 0.0f,
                                       speed_feedback, 0.001f, &target_speed);
    Algorithm_PID_UpdateByFeedbackRate(speed_cfg, &speed_state, target_speed,
                                       speed_feedback, 0.0f, 0.001f, &torque);
    return torque;
}

/**
 * @brief 折算所依赖的性质：纯 P 串级把 (kp_a×k, 外环限幅×k, kp_s/k) 配上反馈×k，
 * 输出力矩逐点相同——【包括外环饱和的时候】。
 *
 * ⚠ 这条钉的是 PID 实现的线性性。将来要是给 PID 加了死区、或者对 feedbackRate
 * 单独限幅，这个等式就不成立了，按比例折算也就不再是行为中性的。
 */
static void test_vmax_rescale_is_behavior_neutral(void)
{
    const float k = APP_DM_VEL_MAX / VMAX_HISTORY;
    const algorithm_pid_config_t angle_old = {.kp = 10.0f, .outputLimit = 100.0f};
    const algorithm_pid_config_t speed_old = {.kp = 4.0f, .outputLimit = 20.0f};
    const algorithm_pid_config_t angle_new = {.kp = 10.0f * k, .outputLimit = 100.0f * k};
    const algorithm_pid_config_t speed_new = {.kp = 4.0f / k, .outputLimit = 20.0f};
    /*
     * 最后一组角度误差大到让外环饱和。⚠ 光饱和外环还不够：内环力矩限幅 ±20 会把差别
     * 一起盖掉，所以速度里要有一组【贴着外环饱和目标】的值（真实 ~150 rad/s），
     * 让内环误差落在线性区——只有这时"只折 kp 不折限幅"才会露馅。
     */
    const float errors[] = {0.0f, 0.3f, -0.8f, 1.5f, 40.0f};
    const float speeds[] = {0.0f, 2.0f, -6.0f, 12.0f, 145.0f, 150.0f, 155.0f};
    unsigned i;
    unsigned j;

    for (i = 0U; i < sizeof(errors) / sizeof(errors[0]); i++)
    {
        for (j = 0U; j < sizeof(speeds) / sizeof(speeds[0]); j++)
        {
            float true_speed = speeds[j];
            float torque_old = joint_cascade(&angle_old, &speed_old, errors[i],
                                             true_speed / k);
            float torque_new = joint_cascade(&angle_new, &speed_new, errors[i],
                                             true_speed);

            assert(fabsf(torque_new - torque_old) <= 1.0e-4f);
        }
    }

    /* 高度环只有 kd 吃速度（feedbackRate = d_H），kd/k 配 d_H×k 输出不变。 */
    {
        const algorithm_pid_config_t height_old = {.kp = 1500.0f, .kd = 150.0f,
                                                   .outputLimit = 100.0f};
        const algorithm_pid_config_t height_new = {.kp = 1500.0f, .kd = 150.0f / k,
                                                   .outputLimit = 100.0f};

        for (j = 0U; j < sizeof(speeds) / sizeof(speeds[0]); j++)
        {
            algorithm_pid_state_t st_old;
            algorithm_pid_state_t st_new;
            float f_old = 0.0f;
            float f_new = 0.0f;
            /* 0~1.5 m/s，跨过常见死区宽度，PID 若加了死区会在这里露馅。 */
            float true_d_H = speeds[j] * 0.01f;

            Algorithm_PID_Init(&st_old);
            Algorithm_PID_Init(&st_new);
            Algorithm_PID_UpdateByFeedbackRate(&height_old, &st_old, 0.20f, 0.19f,
                                               true_d_H / k, 0.001f, &f_old);
            Algorithm_PID_UpdateByFeedbackRate(&height_new, &st_new, 0.20f, 0.19f,
                                               true_d_H, 0.001f, &f_new);
            assert(fabsf(f_new - f_old) <= 1.0e-4f);
        }
    }
}

/**
 * @brief 本次修正的数值校核：自救串级的 3 个数折算，ZJU 抄来的 3 个落地阈值不折算。
 *
 * ⚠ 这是一条【检查点】，不是永久约束：以后在实机上有意重调这几个参数时，
 * 这里跟着改或者整条删掉都是对的。它存在的意义是防止折算那一刻算错或漏改。
 */
static void test_vmax_rescaled_config_values(void)
{
    const float k = APP_DM_VEL_MAX / VMAX_HISTORY;
    const float tol = 1.0e-3f;

    assert(fabsf(APP_DM_VEL_MAX - 45.0f) < 1.0e-6f);
    assert(fabsf(APP_DM_VEL_MIN + 45.0f) < 1.0e-6f);

    /*
     * leg_length_pid.kd 故意【不在这里校核】：修正提交里它按 HEAD 验证过的 300 折成 200，
     * 但这一项正在实机上调（工作区里是没上过机的试验值），钉死它只会跟调参打架。
     * 它依赖的折算性质由上面 test_vmax_rescale_is_behavior_neutral 的高度环那段保证。
     */
    assert(fabsf(Chassis_Config.recovery.joint_angle_pid.kp - 10.0f * k) < tol);
    assert(fabsf(Chassis_Config.recovery.joint_angle_pid.outputLimit - 100.0f * k) < tol);
    assert(fabsf(Chassis_Config.recovery.joint_speed_pid.kp - 4.0f / k) < tol);
    /* 力矩限幅不折算：串级输出的力矩本身不变。 */
    assert(fabsf(Chassis_Config.recovery.joint_speed_pid.outputLimit - 20.0f) < tol);

    /*
     * ⚠ 落地三阈值【故意不折算】：它们是 ZJU 的物理单位原值，不是对着本车旧读数调的。
     * 修正让它们第一次按原意生效。折算了反而离 ZJU 更远——这三条就是拦那个手滑的。
     */
    assert(fabsf(Chassis_Config.observer.land_d_L0_reverse - (-0.05f)) < tol);
    assert(fabsf(Chassis_Config.observer.land_d_L0_peak_min - 0.10f) < tol);
    assert(fabsf(Chassis_Config.observer.land_d_L0_drop - 0.15f) < tol);
}

int main(void)
{
    Chassis_Leg_Config_t config = {
        .geometry = {
            .l1 = 0.215f,
            .l2 = 0.258f,
            .l3 = 0.258f,
            .l4 = 0.215f,
            .l5 = 0.0f,
        },
        .joint = {
            [CHASSIS_JOINT_PHI1] = {
                .angle_offset_rad = CHASSIS_PI,
                .scale = -1.0f,
                .ratio = 1.0f,
            },
            [CHASSIS_JOINT_PHI4] = {
                .angle_offset_rad = 0.0f,
                .scale = -1.0f,
                .ratio = 1.0f,
            },
        },
    };
    Chassis_Leg_t leg;
    Chassis_Leg_t inverse_leg;
    VMC_Torque_t torque;
    VMC_Force_t force;
    VMC_Joint_Target_t target;
    Chassis_Leg_Config_t boundary_config;
    Chassis_Leg_Config_t ratio_config;
    Chassis_Leg_t boundary_leg;
    float phi1_motor_position_rad;
    float phi4_motor_position_rad;

    test_geometry_convention(&config);
    ratio_config = config;
    ratio_config.joint[CHASSIS_JOINT_PHI1].ratio = 0.75f;
    ratio_config.joint[CHASSIS_JOINT_PHI4].ratio = 0.75f;
    test_jacobian_and_virtual_work(&ratio_config);
    test_downward_phi0_branch(&ratio_config);

    VMC_State_Calc(&config, 0.5f, -0.8f, 0.7f, -0.4f, &leg);

    assert(leg.valid_flag == 1U);
    assert(fabsf(leg.phi1 - 2.64159265f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi2 - 0.99543859f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi3 - 2.44615406f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi4 - 0.8f) < TEST_TOLERANCE);
    assert(fabsf(leg.L0 - 0.32316671f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi0 - 1.72079633f) < TEST_TOLERANCE);
    assert(fabsf(leg.d_L0 - 0.15758435f) < TEST_TOLERANCE);
    assert(fabsf(leg.d_phi0 + 0.15f) < TEST_TOLERANCE);

    assert(VMC_Torque_Calc(&config, &leg, -30.0f, 1.2f, &torque) == 1U);
    assert(fabsf(torque.T1 + 4.89775487f) < TEST_TOLERANCE);
    assert(fabsf(torque.T4 - 3.69775487f) < TEST_TOLERANCE);
    assert(VMC_Force_Calc(&config,
                                &leg,
                                torque.T1,
                                torque.T4,
                                &force) == 1U);
    assert(fabsf(force.F0 + 30.0f) < TEST_TOLERANCE);
    assert(fabsf(force.Tp - 1.2f) < TEST_TOLERANCE);

    assert(VMC_Inverse_Calc(&config,
                               &leg,
                               0.15f,
                               CHASSIS_HALF_PI,
                               &target) == 1U);
    phi1_motor_position_rad =
        (target.phi1 -
         config.joint[CHASSIS_JOINT_PHI1].angle_offset_rad) /
        (config.joint[CHASSIS_JOINT_PHI1].scale *
         config.joint[CHASSIS_JOINT_PHI1].ratio);
    phi4_motor_position_rad =
        (target.phi4 -
         config.joint[CHASSIS_JOINT_PHI4].angle_offset_rad) /
        (config.joint[CHASSIS_JOINT_PHI4].scale *
         config.joint[CHASSIS_JOINT_PHI4].ratio);
    VMC_State_Calc(&config,
                   phi1_motor_position_rad,
                   phi4_motor_position_rad,
                   0.0f,
                   0.0f,
                   &inverse_leg);
    assert(fabsf(inverse_leg.L0 - 0.15f) < TEST_TOLERANCE);
    assert(fabsf(inverse_leg.phi0 - CHASSIS_HALF_PI) < TEST_TOLERANCE);

    /* 0.05 m人为门槛已删除，真实二连杆最小可达距离约为0.043 m。 */
    assert(VMC_Inverse_Calc(&config,
                               &inverse_leg,
                               0.045f,
                               CHASSIS_HALF_PI,
                               &target) == 1U);
    assert(VMC_Inverse_Calc(&config,
                               &inverse_leg,
                               0.042f,
                               CHASSIS_HALF_PI,
                               &target) == 0U);

    config.geometry.l5 = 0.06f;
    assert(VMC_Inverse_Calc(&config,
                               &inverse_leg,
                               0.35f,
                               1.40f,
                               &target) == 1U);
    phi1_motor_position_rad =
        (target.phi1 -
         config.joint[CHASSIS_JOINT_PHI1].angle_offset_rad) /
        (config.joint[CHASSIS_JOINT_PHI1].scale *
         config.joint[CHASSIS_JOINT_PHI1].ratio);
    phi4_motor_position_rad =
        (target.phi4 -
         config.joint[CHASSIS_JOINT_PHI4].angle_offset_rad) /
        (config.joint[CHASSIS_JOINT_PHI4].scale *
         config.joint[CHASSIS_JOINT_PHI4].ratio);
    VMC_State_Calc(&config,
                   phi1_motor_position_rad,
                   phi4_motor_position_rad,
                   0.0f,
                   0.0f,
                   &inverse_leg);
    assert(fabsf(inverse_leg.L0 - 0.35f) < TEST_TOLERANCE);
    assert(fabsf(inverse_leg.phi0 - 1.40f) < TEST_TOLERANCE);

    assert(VMC_Inverse_Calc(&config,
                               &inverse_leg,
                               0.60f,
                               CHASSIS_HALF_PI,
                               &target) == 0U);
    assert(target.phi1 == 0.0f);
    assert(target.phi4 == 0.0f);

    /* 相切两圆的位置解存在，但从动杆共线时速度雅可比不可逆。 */
    boundary_config = config;
    boundary_config.geometry.l1 = 0.1f;
    boundary_config.geometry.l2 = 0.1f;
    boundary_config.geometry.l3 = 0.1f;
    boundary_config.geometry.l4 = 0.1f;
    boundary_config.geometry.l5 = 0.2f;
    boundary_config.joint[CHASSIS_JOINT_PHI1].angle_offset_rad = 0.0f;
    boundary_config.joint[CHASSIS_JOINT_PHI1].scale = 1.0f;
    boundary_config.joint[CHASSIS_JOINT_PHI4].angle_offset_rad = 0.0f;
    boundary_config.joint[CHASSIS_JOINT_PHI4].scale = 1.0f;
    VMC_State_Calc(&boundary_config,
                  CHASSIS_HALF_PI,
                  CHASSIS_HALF_PI,
                  0.0f,
                  0.0f,
                  &boundary_leg);
    assert(boundary_leg.valid_flag == 0U);
    assert(fabsf(boundary_leg.L0 - 0.1f) < TEST_TOLERANCE);
    assert(fabsf(boundary_leg.phi0 - CHASSIS_HALF_PI) < TEST_TOLERANCE);
    assert(boundary_leg.d_L0 == 0.0f);
    assert(boundary_leg.d_phi0 == 0.0f);
    assert(VMC_Torque_Calc(&boundary_config,
                          &boundary_leg,
                          -30.0f,
                          1.2f,
                          &torque) == 0U);
    assert(VMC_Force_Calc(&boundary_config,
                                &boundary_leg,
                                1.0f,
                                1.0f,
                                &force) == 0U);
    assert(torque.T1 == 0.0f);
    assert(torque.T4 == 0.0f);

    /* 两圆明显无交点时保留主动杆角，未定义的派生几何量保持零。 */
    boundary_config.geometry.l5 = 0.21f;
    VMC_State_Calc(&boundary_config,
                  CHASSIS_HALF_PI,
                  CHASSIS_HALF_PI,
                  0.0f,
                  0.0f,
                  &boundary_leg);
    assert(boundary_leg.valid_flag == 0U);
    assert(fabsf(boundary_leg.phi1 - CHASSIS_HALF_PI) < TEST_TOLERANCE);
    assert(fabsf(boundary_leg.phi4 - CHASSIS_HALF_PI) < TEST_TOLERANCE);
    assert(boundary_leg.phi2 == 0.0f);
    assert(boundary_leg.L0 == 0.0f);

    test_vmax_rescale_is_behavior_neutral();
    test_vmax_rescaled_config_values();

    return 0;
}
