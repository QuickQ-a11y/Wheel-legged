#include "chassis_control.h"

#include "Limit.h"

#include <math.h>
#include <string.h>

#define CHASSIS_OBS_EPS 1.0e-6f

/**
 * @brief 一阶低通，时间常数或周期非正时直接透传当前值。
 */
static float Observer_Filter(float value,
                             float previous,
                             float time_constant_s,
                             float dt)
{
    float ratio;

    if ((time_constant_s <= 0.0f) || (dt <= 0.0f))
    {
        return value;
    }
    ratio = dt / (time_constant_s + dt);
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }
    return previous + ratio * (value - previous);
}

/**
 * @brief 清空观测量并标记当前使用标称模型参数。
 */
void Chassis_Observer_Init(Chassis_Observer_t *observer)
{
    memset(observer, 0, sizeof(*observer));
    observer->nominal_model_flag = 1U;
}

/**
 * @brief 由轮速与整车偏航残差判定单侧打滑，带进入和退出双时间迟滞。
 */
static void Observer_Slip_Update(
    const Chassis_Observer_Config_t *config,
    const Chassis_Wheel_Config_t *wheel_config,
    const Chassis_t *chassis,
    Chassis_Observer_t *observer)
{
    float track_m = 2.0f * wheel_config->half_track;
    uint32_t side;

    if (track_m > CHASSIS_OBS_EPS)
    {
        observer->wheel_yaw_rate_radps =
            (chassis->body.side_speed[CHASSIS_RIGHT] -
             chassis->body.side_speed[CHASSIS_LEFT]) / track_m;
    }
    else
    {
        observer->wheel_yaw_rate_radps = 0.0f;
    }
    observer->yaw_residual_radps =
        observer->wheel_yaw_rate_radps - chassis->body.d_fai;
    observer->yaw_residual_filtered_radps = Observer_Filter(
        observer->yaw_residual_radps,
        observer->yaw_residual_filtered_radps,
        config->residual_filter_s,
        chassis->dt);

    observer->expected_side_speed_mps[CHASSIS_LEFT] =
        chassis->body.d_s -
        wheel_config->half_track * chassis->body.d_fai;
    observer->expected_side_speed_mps[CHASSIS_RIGHT] =
        chassis->body.d_s +
        wheel_config->half_track * chassis->body.d_fai;

    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        observer->wheel_residual_mps[side] =
            chassis->body.side_speed[side] -
            observer->expected_side_speed_mps[side];
        observer->wheel_residual_filtered_mps[side] =
            Observer_Filter(
                observer->wheel_residual_mps[side],
                observer->wheel_residual_filtered_mps[side],
                config->residual_filter_s,
                chassis->dt);
        if (observer->init_flag == 0U)
        {
            observer->delta_residual_mps[side] = 0.0f;
        }
        else
        {
            observer->delta_residual_mps[side] =
                (chassis->body.side_speed[side] -
                 observer->last_side_speed[side]) -
                chassis->body.dd_s * chassis->dt;
        }

        observer->slip_candidate_flag[side] =
            ((fabsf(observer->wheel_residual_filtered_mps[side]) >=
              config->slip_speed_enter_mps) &&
             ((fabsf(observer->yaw_residual_filtered_radps) >=
               config->slip_yaw_enter_radps) ||
              (fabsf(observer->delta_residual_mps[side]) >=
               config->slip_delta_enter_mps))) ? 1U : 0U;

        if (observer->slip_flag[side] == 0U)
        {
            observer->slip_exit_elapsed_s[side] = 0.0f;
            if (observer->slip_candidate_flag[side] != 0U)
            {
                observer->slip_enter_elapsed_s[side] += chassis->dt;
                if (observer->slip_enter_elapsed_s[side] >=
                    config->slip_enter_s)
                {
                    observer->slip_flag[side] = 1U;
                    observer->slip_enter_elapsed_s[side] = 0.0f;
                }
            }
            else
            {
                observer->slip_enter_elapsed_s[side] = 0.0f;
            }
        }
        else
        {
            observer->slip_enter_elapsed_s[side] = 0.0f;
            if ((fabsf(observer->wheel_residual_filtered_mps[side]) <=
                 config->slip_speed_exit_mps) &&
                (fabsf(observer->yaw_residual_filtered_radps) <=
                 config->slip_yaw_exit_radps) &&
                (fabsf(observer->delta_residual_mps[side]) <=
                 config->slip_delta_exit_mps))
            {
                observer->slip_exit_elapsed_s[side] += chassis->dt;
                if (observer->slip_exit_elapsed_s[side] >=
                    config->slip_exit_s)
                {
                    observer->slip_flag[side] = 0U;
                    observer->slip_exit_elapsed_s[side] = 0.0f;
                }
            }
            else
            {
                observer->slip_exit_elapsed_s[side] = 0.0f;
            }
        }
    }
}

/**
 * @brief 由关节反馈力矩反解支撑力并判定单腿离地与落地。
 */
static void Observer_Force_Update(
    const Chassis_Observer_Config_t *config,
    const Chassis_Leg_Config_t leg_config[CHASSIS_LEG_COUNT],
    const Chassis_t *chassis,
    Chassis_Observer_t *observer)
{
    float total_mass_kg = config->body_mass_kg +
                          2.0f * config->leg_mass_kg +
                          2.0f * config->wheel_mass_kg;
    uint32_t side;

    observer->nominal_static_load_n =
        0.5f * total_mass_kg * config->gravity_mps2;
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        float dd_L0_raw = 0.0f;
        float dd_theta_raw = 0.0f;
        float leg_accel;
        float Fn;
        uint8_t phi1_index =
            leg_config[side].joint[CHASSIS_JOINT_PHI1].motor_index;
        uint8_t phi4_index =
            leg_config[side].joint[CHASSIS_JOINT_PHI4].motor_index;

        if (chassis->dt > CHASSIS_OBS_EPS)
        {
            dd_L0_raw =
                (chassis->leg[side].d_L0 -
                 observer->last_d_L0[side]) /
                chassis->dt;
            dd_theta_raw =
                (chassis->leg[side].d_theta -
                 observer->last_d_theta[side]) /
                chassis->dt;
        }
        observer->dd_L0[side] = Observer_Filter(
            dd_L0_raw,
            observer->dd_L0[side],
            config->residual_filter_s,
            chassis->dt);
        observer->dd_theta[side] = Observer_Filter(
            dd_theta_raw,
            observer->dd_theta[side],
            config->residual_filter_s,
            chassis->dt);

        observer->force_valid_flag[side] = VMC_Force_Calc(
            &leg_config[side],
            &chassis->leg[side],
            chassis->dm_motor[phi1_index].torque_nm,
            chassis->dm_motor[phi4_index].torque_nm,
            &observer->feedback_force[side]);
        if ((observer->force_valid_flag[side] == 0U) ||
            (fabsf(chassis->leg[side].L0) <= CHASSIS_OBS_EPS))
        {
            observer->off_candidate_flag[side] = 0U;
            observer->off_ground_flag[side] = 0U;
            observer->off_elapsed_s[side] = 0.0f;
            observer->land_elapsed_s[side] = 0.0f;
            continue;
        }

        leg_accel =
            chassis->imu.body_accel[Chassis_Config.imu.vertical_accel_axis] -
            observer->dd_L0[side] *
                cosf(chassis->leg[side].theta) +
            2.0f * chassis->leg[side].d_L0 *
                chassis->leg[side].d_theta *
                sinf(chassis->leg[side].theta) +
            chassis->leg[side].L0 *
                observer->dd_theta[side] *
                sinf(chassis->leg[side].theta) +
            chassis->leg[side].L0 *
                chassis->leg[side].d_theta *
                chassis->leg[side].d_theta *
                cosf(chassis->leg[side].theta);
        Fn =
            -(observer->feedback_force[side].F0 *
                  cosf(chassis->leg[side].theta) +
              observer->feedback_force[side].Tp *
                  sinf(chassis->leg[side].theta) /
                  chassis->leg[side].L0) +
            config->leg_mass_kg *
                (config->gravity_mps2 + leg_accel);
        observer->Fn_raw[side] = Fn;
        if (observer->force_init_flag[side] == 0U)
        {
            observer->Fn[side] = Fn;
            observer->force_init_flag[side] = 1U;
        }
        else
        {
            observer->Fn[side] =
                Observer_Filter(
                    Fn,
                    observer->Fn[side],
                    config->normal_force_filter_s,
                    chassis->dt);
        }
        if (observer->nominal_static_load_n > CHASSIS_OBS_EPS)
        {
            observer->Fn_ratio[side] =
                observer->Fn[side] /
                observer->nominal_static_load_n;
        }
        else
        {
            observer->Fn_ratio[side] = 0.0f;
        }

        observer->off_candidate_flag[side] =
            (observer->Fn_ratio[side] <=
             config->off_force_ratio) ? 1U : 0U;
        if (observer->off_ground_flag[side] == 0U)
        {
            observer->land_elapsed_s[side] = 0.0f;
            if (observer->off_candidate_flag[side] != 0U)
            {
                observer->off_elapsed_s[side] += chassis->dt;
                if (observer->off_elapsed_s[side] >= config->off_hold_s)
                {
                    observer->off_ground_flag[side] = 1U;
                    observer->off_elapsed_s[side] = 0.0f;
                }
            }
            else
            {
                observer->off_elapsed_s[side] = 0.0f;
            }
        }
        else
        {
            observer->off_elapsed_s[side] = 0.0f;
            if (observer->Fn_ratio[side] >=
                config->land_force_ratio)
            {
                observer->land_elapsed_s[side] += chassis->dt;
                if (observer->land_elapsed_s[side] >= config->land_hold_s)
                {
                    observer->off_ground_flag[side] = 0U;
                    observer->land_elapsed_s[side] = 0.0f;
                }
            }
            else
            {
                observer->land_elapsed_s[side] = 0.0f;
            }
        }
    }
    observer->all_off_flag =
        ((observer->off_ground_flag[CHASSIS_LEFT] != 0U) &&
         (observer->off_ground_flag[CHASSIS_RIGHT] != 0U)) ? 1U : 0U;
}

/**
 * @brief 由横向加速度估算转向所需的左右支撑力差，并按标称静载限幅。
 */
static void Observer_Turn_Update(
    const Chassis_Observer_Config_t *config,
    const Chassis_Wheel_Config_t *wheel_config,
    const Chassis_t *chassis,
    Chassis_Observer_t *observer)
{
    float total_mass_kg = config->body_mass_kg +
                          2.0f * config->leg_mass_kg +
                          2.0f * config->wheel_mass_kg;
    float force_limit = observer->nominal_static_load_n *
                        config->turn_force_limit_ratio;
    float denominator = 2.0f * wheel_config->half_track;

    observer->lateral_accel_imu_mps2 =
        chassis->imu.body_accel[Chassis_Config.imu.lateral_accel_axis];
    observer->lateral_accel_kinematic_mps2 =
        chassis->body.d_s * chassis->body.d_fai;
    observer->lateral_accel_filtered_mps2 = Observer_Filter(
        observer->lateral_accel_imu_mps2,
        observer->lateral_accel_filtered_mps2,
        config->turn_filter_s,
        chassis->dt);
    observer->lateral_accel_kin_filtered_mps2 = Observer_Filter(
        observer->lateral_accel_kinematic_mps2,
        observer->lateral_accel_kin_filtered_mps2,
        config->turn_filter_s,
        chassis->dt);
    observer->lateral_accel_residual_mps2 =
        observer->lateral_accel_filtered_mps2 -
        observer->lateral_accel_kin_filtered_mps2;
    observer->nominal_cg_height_m =
        0.5f * (chassis->leg[CHASSIS_LEFT].L0 +
                chassis->leg[CHASSIS_RIGHT].L0) +
        config->body_cg_to_hip_m;

    if (fabsf(denominator) > CHASSIS_OBS_EPS)
    {
        observer->turn_support_imu_raw_n =
            total_mass_kg * observer->nominal_cg_height_m *
            observer->lateral_accel_filtered_mps2 / denominator;
        observer->turn_support_kin_raw_n =
            total_mass_kg * observer->nominal_cg_height_m *
            observer->lateral_accel_kin_filtered_mps2 / denominator;
    }
    else
    {
        observer->turn_support_imu_raw_n = 0.0f;
        observer->turn_support_kin_raw_n = 0.0f;
    }
    observer->turn_support_imu_limited_n = Algorithm_LimitSymmetric(
        observer->turn_support_imu_raw_n,
        force_limit);
    observer->turn_support_kin_limited_n = Algorithm_LimitSymmetric(
        observer->turn_support_kin_raw_n,
        force_limit);
}

/**
 * @brief 更新打滑、离地和转向支撑力三组只读观测量。
 *
 * 只写 Chassis.observer，不回写任何控制量；首轮先用当前反馈建立差分基准。
 */
void Chassis_Observer_Update(const Chassis_Config_t *config,
                             Chassis_t *chassis)
{
    Chassis_Observer_t *observer = &chassis->observer;
    uint32_t side;

    if (observer->init_flag == 0U)
    {
        for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
        {
            observer->last_side_speed[side] =
                chassis->body.side_speed[side];
            observer->last_d_L0[side] =
                chassis->leg[side].d_L0;
            observer->last_d_theta[side] =
                chassis->leg[side].d_theta;
        }
    }

    Observer_Slip_Update(&config->observer,
                                 &config->wheel,
                                 chassis,
                                 observer);
    Observer_Force_Update(&config->observer,
                                  config->leg,
                                  chassis,
                                  observer);
    Observer_Turn_Update(&config->observer,
                                 &config->wheel,
                                 chassis,
                                 observer);
    for (side = 0U; side < CHASSIS_LEG_COUNT; side++)
    {
        observer->last_side_speed[side] =
            chassis->body.side_speed[side];
        observer->last_d_L0[side] =
            chassis->leg[side].d_L0;
        observer->last_d_theta[side] =
            chassis->leg[side].d_theta;
    }
    observer->init_flag = 1U;
}
