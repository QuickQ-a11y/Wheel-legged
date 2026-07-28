#include "chassis_vmc.h"

#include <assert.h>
#include <math.h>

#define TEST_TOLERANCE 2.0e-5f

int main(void)
{
    const chassis_leg_config_t config = {
        .geometry = {
            .link1_m = 0.215f,
            .link2_m = 0.258f,
            .link3_m = 0.258f,
            .link4_m = 0.215f,
            .frame_joint_distance_m = 0.0f,
            .min_leg_length_m = 0.05f,
        },
        .joint = {
            [CHASSIS_JOINT_FRONT] = {
                .angle_offset_rad = CHASSIS_PI,
                .angle_scale = -1.0f,
                .torque_scale = -1.0f,
            },
            [CHASSIS_JOINT_BACK] = {
                .angle_offset_rad = 0.0f,
                .angle_scale = -1.0f,
                .torque_scale = -1.0f,
            },
        },
    };
    chassis_vmc_state_t leg;
    chassis_vmc_torque_t torque;

    vmc_calc_state(&config, 0.5f, -0.8f, 0.7f, -0.4f, &leg);

    assert(fabsf(leg.phi1_rad - 2.64159265f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi2_rad - 0.99543859f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi3_rad - 2.44615406f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi4_rad - 0.8f) < TEST_TOLERANCE);
    assert(fabsf(leg.length_m - 0.32316671f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi0_rad - 1.72079633f) < TEST_TOLERANCE);
    assert(fabsf(leg.length_speed_mps - 0.15758435f) < TEST_TOLERANCE);
    assert(fabsf(leg.phi0_speed_radps + 0.15f) < TEST_TOLERANCE);

    vmc_calc_torque(&config, &leg, -30.0f, 1.2f, &torque);
    assert(fabsf(torque.front_nm + 4.89775487f) < TEST_TOLERANCE);
    assert(fabsf(torque.back_nm - 3.69775487f) < TEST_TOLERANCE);

    return 0;
}
