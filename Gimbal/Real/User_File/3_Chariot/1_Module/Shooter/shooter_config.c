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
     * 量纲推算：M2006 空载转子转速约 17000 rpm（C610 手册性能曲线上的
     * 输出轴 470 rpm × 36:1）。⚠ 摩擦轮的 P36 减速箱【已经物理拆掉】，
     * 所以这个转子转速就是轮子能到的转速。满量程电流 10000，
     * 取 kp 使几百 rpm 的误差就能给出上千的电流，即 kp 在 1 附近；
     * ki 先给小值靠积分消静差。
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
     * ⚠⚠ 占位值，【未经实机整定】。摩擦轮减速箱已拆，所以 5000 就是轮速
     * 5000 rpm，量级上对得起 17mm 摩擦轮；但它和实际需要的弹速对不对得上
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
#if APP_TRIGGER_MOTOR == APP_TRIGGER_LK4005
    /*
     * 翎控速度环，【全部未经实机整定】。输出是 0xA1 的 iq 原始值（协议范围 ±2000）。
     *
     * 起步量级按"每 rad/s 误差给多少安"与下面 M2006 那组取同值（kp 3 A、ki 0.1 A），
     * 再乘 APP_LK_IQ_PER_A 换成翎控计数——两台电机的减速比和转矩常数都不同，
     * 这只是个不离谱的起点。integralLimit 同样按积分状态单位：I 项限到满量程一半。
     */
    .trigger_speed_pid = {
        .kp = 3.0f * APP_LK_IQ_PER_A,
        .ki = 0.1f * APP_LK_IQ_PER_A,
        .kd = 0.0f, /* 与摩擦轮同理：没有可信的角加速度来源，退化成 PI。 */
        .integralLimit = ((float)APP_LK_IQ_MAX / 2.0f) / (0.1f * APP_LK_IQ_PER_A),
        .outputLimit = (float)APP_LK_IQ_MAX,
    },
    /*
     * 拨盘力矩限幅默认 3 A，【未经实机整定】：MG40-50 手册里驱动板 DG40R/C7 额定 6 A、
     * 峰值 8 A，先给额定的一半。⚠ "A"的真假取决于 APP_LK_IQ_PER_A，那个数还没核对。
     */
    .trigger_default_I_limit = 3.0f,
#else
    /*
     * 速度环 kp 的量纲推算：误差约 3 rad/s 时给满量程电流 10000，故 kp ≈ 3000；
     * ki 先给小值靠积分消静差。
     *
     * ⚠ integralLimit 限的是【积分状态】（误差 × s，这里是 rad），不是电流：公共 PID 的
     * I 项 = ki × 积分状态（PID.c）。要把 I 项限到满量程一半，就得是 5000 / ki = 50。
     * 原来按电流口径填了 5000，实际允许 I 项到 50 万，等于没有限。
     */
    .trigger_speed_pid = {
        .kp = 3000.0f,
        .ki = 100.0f,
        .kd = 0.0f, /* 与摩擦轮同理：没有可信的角加速度来源，退化成 PI。 */
        .integralLimit = (float)(APP_DJI_CURRENT_MAX / 2) / 100.0f,
        .outputLimit = (float)APP_DJI_CURRENT_MAX,
    },
    /*
     * 拨盘力矩限幅默认 5 A，【未经实机整定】：M2006 在 C610 上最多 10 A，
     * 先给一半。上机时逐步往下调到"故意卡弹时顶不碎弹丸"为止。
     */
    .trigger_default_I_limit = 5.0f,
#endif
    /* 到位容差取一发的 5%（8 槽拨叉约 2.25°），【未经实机整定】。 */
    .trigger_arrive_rad = APP_SHOOT_RAD_PER_AMMO * 0.05f,
    /*
     * 防卡弹阈值，【全部未经实机整定】。
     *
     * 电流饱和取力矩限幅的 95%，与 RoboWalker 的 9.5 A / 10 A 同比例。
     * 两个时长直接取自 RoboWalker（crt_booster.h:89,91）。
     * 退弹角按【一发的 30%】给，与 RoboWalker 的 π/12 对 2π/7 同比例，
     * 好处是换拨叉改 APP_SHOOT_AMMO_PER_ROUND 时它自动跟着缩放。
     */
    .jam_current_ratio = 0.95f,
    .jam_confirm_s = 0.5f,
    .jam_back_s = 0.3f,
    .jam_back_rad = APP_SHOOT_RAD_PER_AMMO * 0.3f,
    .jam_retry_max = 3U,
    .default_dt = APP_CTRL_DT_S,
};
