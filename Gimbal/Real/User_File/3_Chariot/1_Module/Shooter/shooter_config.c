#include "shooter_config.h"

const Shooter_Config_t Shooter_Config = {
    /*
     * ⚠ 两个门都从 0 出厂。发射机构未装机时打开 fric_flag 会让 FDCAN2 陷入
     * BusOff 循环（总线上没有节点应答 ACK），理由见 shooter_config.h。
     * 实机按 Codex文档 的三步走：先都关着只看反馈，再开拨盘，最后开摩擦轮。
     */
    .output = {
        .fric_flag = 0U,
        .trigger_flag = 0U,
    },
    /*
     * 摩擦轮速度环的起步参数，【未经实机整定】。
     *
     * 量纲推算：M2006 空载转子转速约 17000 rpm（输出轴 470 rpm × 36:1 减速比，
     * 见 C610 手册的性能曲线），满量程电流 10000。取 kp 使几百 rpm 的误差
     * 就能给出上千的电流，即 kp 在 1 附近；ki 先给小值靠积分消静差。
     *
     * integralLimit 和 outputLimit 都按电流口径：积分单独限到满量程的一半，
     * 防止起转那段长时间大误差把积分灌满、到达目标后迟迟压不下来。
     */
    .fric_speed_pid = {
        .kp = 1.0f,
        .ki = 0.02f,
        .kd = 0.0f, /* 必须为 0，理由见 shooter_config.h */
        .integralLimit = (float)(APP_DJI_CURRENT_MAX / 2),
        .outputLimit = (float)APP_DJI_CURRENT_MAX,
    },
    .fric_ready_rpm = 300.0f,
    .default_dt = APP_CTRL_DT_S,
};
