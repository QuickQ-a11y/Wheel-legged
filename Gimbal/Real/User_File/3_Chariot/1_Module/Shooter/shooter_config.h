#ifndef SHOOTER_CONFIG_H
#define SHOOTER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "PID.h"
#include "app_config.h"

#include <stdint.h>

/**
 * @brief 输出安全门。
 *
 * ⚠ 这两个门是【按总线分】的，不是习惯上的"按功能分"。某条总线上一个节点都没有时，
 * 发出去的帧没人应答 ACK，发送错误计数会爬到 BusOff、驱动 Stop/Start 再 BusOff，
 * 空转在错误恢复里。所以分步装机时必须能单独关掉一条总线的发送。
 */
typedef struct
{
    uint8_t fric_flag;    /* FDCAN2（三个摩擦轮）电流输出与发帧使能。 */
    uint8_t trigger_flag; /* FDCAN3（拨盘）电流输出与发帧使能。 */
} Shooter_Output_Config_t;

typedef struct
{
    Shooter_Output_Config_t output;
    /*
     * 摩擦轮速度环，三路共用一组参数。目标与反馈都用【转子】转速 rpm，
     * 输出是 C610 的控制电流原始值（±APP_DJI_CURRENT_MAX 对应 ±10 A）。
     *
     * ⚠ kd 必须为 0。仓库里唯一的 PID 更新函数是
     * Algorithm_PID_UpdateByFeedbackRate()，它取 derivative = -feedbackRate，
     * 而速度环没有可信的加速度反馈，调用处 feedbackRate 传 0.0f，
     * 整体退化成 PI——摩擦轮速度环本来也只用 PI，不是漏传参数。
     */
    algorithm_pid_config_t fric_speed_pid;
    /*
     * 摩擦轮速度环的在线判据：目标非零时，转子转速与目标差在这个带内
     * 才算转稳。仅供 Watch 和将来的火控判断，不参与控制。
     */
    float fric_ready_rpm;
    /*
     * 摩擦轮运行目标初值（转子 rpm），写进 Shooter.fric_target_rpm。
     * VrB 只决定摩擦轮开不开，转多快仍由那个 Watch 可改的字段说了算，
     * 这样整定期和实战期共用一个入口，不用为改目标重新烧录。
     */
    float fric_default_rpm;

    /*
     * 拨盘串级环。外环位置、内环速度，两环都用【输出轴】口径
     * （rad 和 rad/s），与摩擦轮那套【转子】rpm 不是同一个口径——
     * 一发 = 2π/N 这个关系只在输出轴上成立。
     *
     * ⚠ 位置环是【纯 P】：ki/kd 为 0、调用处 feedbackRate 传 0.0f。
     * 串级的阻尼由内环速度环提供，外环再加微分会和内环打架。
     * ⚠ 位置环的 outputLimit 是【目标角速度】的上限，直接决定连发跟不跟得上
     * APP_SHOOT_BURST_HZ：至少要 BURST_HZ × APP_SHOOT_RAD_PER_AMMO。
     * 连发跟不上时先查这个数，不是先查速度环。
     */
    algorithm_pid_config_t trigger_angle_pid;
    algorithm_pid_config_t trigger_speed_pid;

    /*
     * 防卡弹。判据是【电调堵转码 或 电流饱和】，取或——
     * C610 在反馈 data[7] 直接报堵转，这是 C620 没有的白送诊断；
     * 但手册没写它的响应延迟和灵敏度，所以不拿它当唯一判据。
     */
    float jam_current;      /* 电流饱和阈值，C610 原始口径（±APP_DJI_CURRENT_MAX）。 */
    float jam_confirm_s;    /* 判据持续这么久才确认卡弹，s。 */
    float jam_back_s;       /* 退弹持续时长，s。 */
    float jam_back_rad;     /* 退弹角度，输出轴 rad。 */
    uint32_t jam_retry_max; /* 连续退弹这么多次仍未脱困就放弃并封锁输出。 */

    float default_dt; /* 任务 tick 差越界时回退的周期，单位 s。 */
} Shooter_Config_t;

extern const Shooter_Config_t Shooter_Config;

#ifdef __cplusplus
}
#endif

#endif
