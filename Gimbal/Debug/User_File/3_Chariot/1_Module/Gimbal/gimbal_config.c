#include "gimbal_config.h"

/*
 * 云台全部机械、标定、限幅和输出开关常量集中在这里，
 * 任务层、设备层和控制器都不得再出现硬编码值。
 *
 * TODO 占位符：scale / ratio / zero_rad / angle_min / angle_max 尚未实机标定，
 * 当前是"直连、正方向一致、零位在电机零点、不限位"的中性取值。上实机前必须
 * 逐项标定，否则云台会朝错误方向转或撞限位。
 */
const Gimbal_Config_t Gimbal_Config = {
    .yaw = {
        .scale = 1.0f,
        .ratio = 1.0f,
        .zero_rad = 0.0f,
        .angle_min = -3.1416f,
        .angle_max = 3.1416f,
        .T_limit = 3.0f,
    },
    .pitch = {
        .scale = 1.0f,
        .ratio = 1.0f,
        .zero_rad = 0.0f,
        .angle_min = -0.6f,
        .angle_max = 0.6f,
        .T_limit = 3.0f,
    },
    /* 上电默认全部封锁；实机确认方向和限幅之前不允许打开。 */
    .output = {
        .dm_flag = 0U,
        .dji_flag = 0U,
    },
    .default_dt = APP_CTRL_DT_S,
};
