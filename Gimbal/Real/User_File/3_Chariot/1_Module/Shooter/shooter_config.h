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
    float default_dt; /* 任务 tick 差越界时回退的周期，单位 s。 */
} Shooter_Config_t;

extern const Shooter_Config_t Shooter_Config;

#ifdef __cplusplus
}
#endif

#endif
