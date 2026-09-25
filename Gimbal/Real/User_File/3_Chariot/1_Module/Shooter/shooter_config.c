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
    /*
     * ⚠⚠ 占位值，【未经实机整定】。摩擦轮是 M2006（减速比 36:1），
     * 转子 5000 rpm 只等于输出轴 139 rpm——这个数和实际需要的弹速对不对得上
     * 没有核对过，上机整定时连同 fric_ready_rpm 一起重定。
     */
    .fric_default_rpm = 5000.0f,
    /*
     * 拨盘串级环起步参数，【全部未经实机整定】。
     *
     * 位置环 kp 的量纲推算：一发是 APP_SHOOT_RAD_PER_AMMO（N=8 时 0.785 rad），
     * 连发要求 8 Hz × 0.785 = 6.28 rad/s。取 kp 使"差一发"时正好给出这个角速度，
     * 即 kp ≈ 8。outputLimit 取 20 rad/s，是该需求的 3 倍余量，
     * 同时远低于 M2006 输出轴空载约 49 rad/s。
     */
    .trigger_angle_pid = {
        .kp = 8.0f,
        .ki = 0.0f, /* 位置环不积分：卡住时积分会灌满，脱困瞬间猛冲。 */
        .kd = 0.0f, /* 阻尼由内环速度环给，理由见 shooter_config.h。 */
        .integralLimit = 0.0f,
        .outputLimit = 20.0f,
    },
    /*
     * 速度环 kp 的量纲推算：误差约 3 rad/s 时给满量程电流 10000，故 kp ≈ 3000；
     * ki 先给小值靠积分消静差。积分单独限到满量程一半，
     * 防止起转那段长时间大误差把积分灌满。
     */
    .trigger_speed_pid = {
        .kp = 3000.0f,
        .ki = 100.0f,
        .kd = 0.0f, /* 与摩擦轮同理：没有可信的角加速度来源，退化成 PI。 */
        .integralLimit = (float)(APP_DJI_CURRENT_MAX / 2),
        .outputLimit = (float)APP_DJI_CURRENT_MAX,
    },
    /*
     * 防卡弹阈值，【全部未经实机整定】。
     *
     * jam_current 取满量程的 95%，与 RoboWalker 的 9.5 A / 10 A 同比例。
     * 两个时长直接取自 RoboWalker（crt_booster.h:89,91）。
     * 退弹角按【一发的 30%】给，与 RoboWalker 的 π/12 对 2π/7 同比例，
     * 好处是换拨叉改 APP_SHOOT_AMMO_PER_ROUND 时它自动跟着缩放。
     */
    .jam_current = (float)APP_DJI_CURRENT_MAX * 0.95f,
    .jam_confirm_s = 0.5f,
    .jam_back_s = 0.3f,
    .jam_back_rad = APP_SHOOT_RAD_PER_AMMO * 0.3f,
    .jam_retry_max = 3U,
    .default_dt = APP_CTRL_DT_S,
};
