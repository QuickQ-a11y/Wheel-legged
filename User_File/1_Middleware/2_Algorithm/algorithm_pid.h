#ifndef ALGORITHM_PID_H
#define ALGORITHM_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_status.h"

typedef struct
{
    float kp;                    /* 比例系数。 */
    float ki;                    /* 积分系数。 */
    float kd;                    /* 微分系数。 */
    float integralLimit;         /* 积分项状态限幅，单位由调用方定义。 */
    float outputLimit;           /* 输出限幅，单位由调用方定义。 */
} algorithm_pid_config_t;

typedef struct
{
    float integral;              /* 积分状态，单位由调用方定义。 */
    float lastOutput;            /* 最近一次 PID 输出，单位由调用方定义。 */
} algorithm_pid_state_t;

/**
 * @brief 初始化 PID 状态。
 */
app_status_t Algorithm_PID_Init(algorithm_pid_state_t *state);

/**
 * @brief 使用反馈速度作为阻尼项更新 PID 输出。
 *
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
