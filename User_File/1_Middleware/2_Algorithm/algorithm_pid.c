#include "algorithm_pid.h"

#include <math.h>
#include <string.h>

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

app_status_t Algorithm_PID_Init(algorithm_pid_state_t *state)
{
    if (state == NULL)
    {
        return APP_STATUS_INVALID_PARAM;
    }

    memset(state, 0, sizeof(*state));

    return APP_STATUS_OK;
}

app_status_t Algorithm_PID_UpdateByFeedbackRate(const algorithm_pid_config_t *config,
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
        return APP_STATUS_INVALID_PARAM;
    }

    error = targetValue - feedbackValue;
    derivative = -feedbackRate;
    state->integral += error * dtSec;
    state->integral = Algorithm_PID_LimitSymmetric(state->integral,
                                                   config->integralLimit);

    outputValue = (config->kp * error) +
                  (config->ki * state->integral) +
                  (config->kd * derivative);
    state->lastOutput = Algorithm_PID_LimitSymmetric(outputValue,
                                                     config->outputLimit);
    *output = state->lastOutput;

    return APP_STATUS_OK;
}
