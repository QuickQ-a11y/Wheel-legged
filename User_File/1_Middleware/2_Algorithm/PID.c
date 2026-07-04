#include "PID.h"

#include <math.h>
#include <string.h>

/* PID 输出和积分采用对称限幅，limit 小于等于 0 时表示该项不允许输出。 */
static float Algorithm_PID_LimitFloat(float value, float minValue, float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }

    if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

static float Algorithm_PID_LimitSymmetric(float value, float limit)
{
    float positiveLimit = fabsf(limit);

    if (positiveLimit <= 0.0f)
    {
        return 0.0f;
    }

    return Algorithm_PID_LimitFloat(value, -positiveLimit, positiveLimit);
}

void Algorithm_PID_Init(algorithm_pid_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    memset(state, 0, sizeof(*state));
}

void Algorithm_PID_UpdateByFeedbackRate(const algorithm_pid_config_t *config,
                                        algorithm_pid_state_t *state,
                                        float targetValue,
                                        float feedbackValue,
                                        float feedbackRate,
                                        float dtSec,
                                        float *output)
{
    float error;
    float derivative;
    float outputValue;

    if ((config == NULL) || (state == NULL) || (output == NULL) || (dtSec <= 0.0f))
    {
        return;
    }

    error = targetValue - feedbackValue;

    /* 目标值由上层慢速规划给出时，直接使用反馈速度作为阻尼项可减少目标差分噪声。 */
    derivative = -feedbackRate;

    /* 积分按真实周期累加，避免控制任务周期抖动直接改变积分强度。 */
    state->integral += error * dtSec;
    state->integral = Algorithm_PID_LimitSymmetric(state->integral,
                                                   config->integralLimit);

    outputValue = (config->kp * error) +
                  (config->ki * state->integral) +
                  (config->kd * derivative);
    state->lastOutput = Algorithm_PID_LimitSymmetric(outputValue,
                                                     config->outputLimit);
    *output = state->lastOutput;
}
