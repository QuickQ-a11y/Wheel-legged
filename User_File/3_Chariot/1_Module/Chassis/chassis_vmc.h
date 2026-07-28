#ifndef CHASSIS_VMC_H
#define CHASSIS_VMC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_config.h"

typedef struct
{
    float phi1_rad;
    float phi2_rad;
    float phi3_rad;
    float phi4_rad;
    float length_m;
    float phi0_rad;
    float length_speed_mps;
    float phi0_speed_radps;
} chassis_vmc_state_t;

typedef struct
{
    float front_nm;
    float back_nm;
} chassis_vmc_torque_t;

/**
 * @brief 由前后髋关节反馈计算五连杆虚拟腿状态。
 */
void vmc_calc_state(const chassis_leg_config_t *config,
                    float front_position_rad,
                    float back_position_rad,
                    float front_speed_radps,
                    float back_speed_radps,
                    chassis_vmc_state_t *leg);

/**
 * @brief 将虚拟支撑力和腿摆力矩映射为前后髋关节力矩。
 */
void vmc_calc_torque(const chassis_leg_config_t *config,
                     const chassis_vmc_state_t *leg,
                     float support_force_n,
                     float swing_torque_nm,
                     chassis_vmc_torque_t *torque);

#ifdef __cplusplus
}
#endif

#endif
