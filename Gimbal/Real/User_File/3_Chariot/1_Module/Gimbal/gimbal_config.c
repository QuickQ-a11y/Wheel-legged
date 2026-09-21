#include "gimbal_config.h"

/*
 * 云台全部机械、标定、限幅和输出开关常量集中在这里，
 * 任务层、设备层和控制器都不得再出现硬编码值。
 *
 * ==========================================================================
 * TODO 占位符：以下三类值尚未实机标定，上实机前必须逐项确认。
 *
 * 1) 每轴两个方向符号：imu_scale（IMU 反馈口径）和 motor_scale（电机口径），
 *    外加 stick_rate 的符号（只影响推杆手感，填反不危险）。
 *    前两个都能在两道输出门都关着的状态下靠看 Watch 定出来——手推云台，
 *    看 angle / d_angle / angle_joint 往哪个方向增大。Gimbal_Config 是
 *    const 放在 flash 里，调试器改不了，所以一次上电把符号全读完再重烧。
 *
 * 2) motor_scale 同时决定力矩下发方向（见 Gimbal_Command_Send），填反就是
 *    正反馈飞车。最后一步单轴验证：另一轴 T_limit 置 0 封死，本轴 T_limit
 *    保持最小值，手扶云台小幅推杆，收敛说明对了，发散说明反了。
 *
 * 3) motor_zero / joint_min / joint_max。当前 pitch 限位是按 ±0.5 rad
 *    估的，不是实测机械限位。
 *
 * 4) APP_DM_TOR_MIN/MAX 仍是出厂占位值，而它发送和接收两侧都用到。
 *    和电机实际 TMAX 不一致的话，下发力矩会整体按比例缩放——做力矩标定
 *    之前必须先从调试助手核对。
 * ==========================================================================
 *
 * PID 初值取自 SPR 参考工程在同款 DM4310 + MIT 力矩下的实测参数，量纲一致
 * （rad -> rad/s -> N*m）。注意本工程的 integralLimit 限的是积分状态本身
 * （单位 误差*秒），不是 ki*integral，与 SPR 的 max_iout 语义不同。
 */
const Gimbal_Config_t Gimbal_Config = {
    .yaw = {
        .gyro_index = 2U, /* 绕 Z 轴。 */
        .imu_scale = 1.0f, /* 待实机确认。 */
        .stick_rate = -3.50f,   /* 满杆 3.5 rad/s，取自 SPR 实测手感。 */
        .follow_limit = 0.5f, /* 目标最多超前反馈 0.5 rad，云台被卡住时不会绕死。 */
        /* YAW 轴机械上可以无限转，没有限位。min >= max 即表示不限，控制链路里也不对 YAW 调限位函数。 */
        .angle_min = 0.0f,
        .angle_max = 0.0f,
        .T_limit = 3.50f, /* 第一次通电用的保守值，SPR 实测可到 5.0。 */
        /*
         * 待实机确认。YAW 这个符号比 PITCH 要紧：它还决定下发给底盘的 yaw_rel
         * 极性，填反的话底盘跟随和小陀螺平移方向会一起反。
         */
        .motor_scale = 1.0f,
        .motor_ratio = 1.0f,
        .motor_zero = 0.0f,
        /* 同上，YAW 无机械限位。 */
        .joint_min = 0.0f,
        .joint_max = 0.0f,
        .angle_pid = {
            .kp = 30.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integralLimit = 0.0f,
            .outputLimit = 10.0f, /* 目标角速度上限，rad/s。 */
        },
        .speed_pid = {
            .kp = 1.00f, /* 1 rad/s 误差给 1 N*m。 */
            .ki = 0.0f,
            .kd = 0.0f,
            .integralLimit = 0.0f,
            .outputLimit = 3.50f, /* 与 T_limit 同值。 */
        },
    },
    .pitch = {
        .gyro_index = 1U, /* 绕 Y 轴。 */
        .imu_scale = 1.0f, /* 待实机确认。 */
        .stick_rate = -2.0f,   /* PITCH 行程只有 ±0.5 rad，速率比 YAW 慢一些。 */
        .follow_limit = 0.3f,
        .angle_min = -0.5f,
        .angle_max = 0.5f,
        .T_limit = 3.50f,
        /* 实测：编码器抬头为正，而整车右手系绕 +Y 正转是低头，故取反。 */
        .motor_scale = -1.0f,
        .motor_ratio = 1.0f,
        .motor_zero = 0.0f,
        .joint_min = -0.5f,
        .joint_max = 0.5f,
        .angle_pid = {
            .kp = 20.0f,
            .ki = 0.0f, /* 重力静差先不用积分顶，实机看到静差再加。 */
            .kd = 0.0f,
            .integralLimit = 0.0f,
            .outputLimit = 5.0f,
        },
        .speed_pid = {
            .kp = 1.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integralLimit = 0.0f,
            .outputLimit = 3.50f,
        },
    },
    /* 上电默认全部封锁；实机确认方向和限幅之前不允许打开。 */
    .output = {
        .dm_flag = 1U,
        .dji_flag = 0U,
        /* 板间下发不通电机，默认开；底盘侧 follow.enable_flag 才是动力相关的那道门。 */
        .board_flag = 1U,
    },
    .pitch_home_rate = 1.0f,
    .default_dt = APP_CTRL_DT_S,
};
