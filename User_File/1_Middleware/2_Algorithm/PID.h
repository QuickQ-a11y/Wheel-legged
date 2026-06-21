#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_status.h"

/**
 * @brief PID 参数配置。
 *
 * 公共 PID 不绑定具体物理量，调用方需要保证目标值、反馈值、反馈速度、
 * 积分限幅和输出限幅使用同一套单位定义。
 */
typedef struct
{
    float kp;                    /* 比例系数。 */
    float ki;                    /* 积分系数。 */
    float kd;                    /* 微分系数。 */
    float integralLimit;         /* 积分项状态限幅，单位由调用方定义。 */
    float outputLimit;           /* 输出限幅，单位由调用方定义。 */
} algorithm_pid_config_t;

/**
 * @brief PID 运行状态。
 *
 * 状态由调用方持有，便于同一套 PID 代码服务多个控制环，同时避免算法层维护业务全局状态。
 */
typedef struct
{
    float integral;              /* 积分状态，单位由调用方定义。 */
    float lastOutput;            /* 最近一次 PID 输出，单位由调用方定义。 */
} algorithm_pid_state_t;

/**
 * @brief 初始化 PID 状态。
 *
 * 调用方在控制器初始化阶段调用一次，运行中是否清积分由具体控制状态机决定。
 */
app_status_t Algorithm_PID_Init(algorithm_pid_state_t *state);

/**
 * @brief 使用反馈速度作为阻尼项更新 PID 输出。
 *
 * `dtSec` 是本次更新距离上次更新的实际时间间隔，单位 s，用于积分项计算。
 * derivative = -feedbackRate，适合目标值低频变化、反馈速度可信的控制环。
 */
app_status_t Algorithm_PID_UpdateByFeedbackRate(const algorithm_pid_config_t *config,
                                                algorithm_pid_state_t *state,
                                                float targetValue,
                                                float feedbackValue,
                                                float feedbackRate,
                                                float dtSec,
                                                float *output);

#ifdef __cplusplus
}
#endif

#endif
