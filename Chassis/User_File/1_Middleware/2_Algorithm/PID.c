#include "PID.h"

#include "Limit.h"

#include <math.h>
#include <string.h>

void Algorithm_PID_Init(algorithm_pid_state_t *state)
{
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

    /* dt 非正时积分无定义，本轮不更新控制器。 */
    if (dtSec <= 0.0f)
    {
        return;
    }

    error = targetValue - feedbackValue;

    /* 目标值由上层慢速规划给出时，直接使用反馈速度作为阻尼项可减少目标差分噪声。 */
    derivative = -feedbackRate;

    /* 积分按真实周期累加，避免控制任务周期抖动直接改变积分强度。 */
    state->integral += error * dtSec;
    state->integral = Algorithm_LimitSymmetric(state->integral,
                                               config->integralLimit);

    outputValue = (config->kp * error) +
                  (config->ki * state->integral) +
                  (config->kd * derivative);
    state->lastOutput = Algorithm_LimitSymmetric(outputValue,
                                                 config->outputLimit);
    *output = state->lastOutput;
}
