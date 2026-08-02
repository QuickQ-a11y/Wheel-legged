#ifndef CHASSIS_OBSERVER_H
#define CHASSIS_OBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_config.h"
#include "chassis_vmc.h"

#include <stdint.h>

/** @brief 打滑、离地和转向补偿的只读调试观测状态。 */
typedef struct
{
    uint8_t nominal_model_flag; /* 使用未实测的MATLAB质量参数。 */
    uint8_t init_flag;
    float wheel_yaw_rate_radps;
    float yaw_residual_radps;
    float yaw_residual_filtered_radps;
    float expected_side_speed_mps[CHASSIS_LEG_COUNT];
    float wheel_residual_mps[CHASSIS_LEG_COUNT];
    float wheel_residual_filtered_mps[CHASSIS_LEG_COUNT];
    float delta_residual_mps[CHASSIS_LEG_COUNT];
    float slip_enter_elapsed_s[CHASSIS_LEG_COUNT];
    float slip_exit_elapsed_s[CHASSIS_LEG_COUNT];
    uint8_t slip_candidate_flag[CHASSIS_LEG_COUNT];
    uint8_t slip_flag[CHASSIS_LEG_COUNT];

    VMC_Force_t feedback_force[CHASSIS_LEG_COUNT];
    uint8_t force_valid_flag[CHASSIS_LEG_COUNT];
    float dd_L0[CHASSIS_LEG_COUNT];
    float dd_theta[CHASSIS_LEG_COUNT];
    float Fn_raw[CHASSIS_LEG_COUNT];
    float Fn[CHASSIS_LEG_COUNT];
    float Fn_ratio[CHASSIS_LEG_COUNT];
    float off_elapsed_s[CHASSIS_LEG_COUNT];
    float land_elapsed_s[CHASSIS_LEG_COUNT];
    uint8_t off_candidate_flag[CHASSIS_LEG_COUNT];
    uint8_t off_ground_flag[CHASSIS_LEG_COUNT];
    uint8_t all_off_flag;

    float lateral_accel_imu_mps2;
    float lateral_accel_kinematic_mps2;
    float lateral_accel_filtered_mps2;
    float lateral_accel_kin_filtered_mps2;
    float lateral_accel_residual_mps2;
    float nominal_cg_height_m;
    float nominal_static_load_n;
    float turn_support_imu_raw_n;
    float turn_support_kin_raw_n;
    float turn_support_imu_limited_n;
    float turn_support_kin_limited_n;

    float last_side_speed[CHASSIS_LEG_COUNT];
    float last_d_L0[CHASSIS_LEG_COUNT];
    float last_d_theta[CHASSIS_LEG_COUNT];
    uint8_t force_init_flag[CHASSIS_LEG_COUNT];
} Chassis_Observer_t;

typedef struct Chassis Chassis_t;

void Chassis_Observer_Init(Chassis_Observer_t *observer);

/** @brief 直接消费本轮底盘状态，更新只读打滑、离地和转向观测量。 */
void Chassis_Observer_Update(const Chassis_Config_t *config,
                             Chassis_t *chassis);

#ifdef __cplusplus
}
#endif

#endif
