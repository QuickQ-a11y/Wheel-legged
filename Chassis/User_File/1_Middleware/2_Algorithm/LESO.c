#include "LESO.h"

#include "Limit.h"

#include <string.h>

void Algorithm_LESO_Init(algorithm_leso_t *leso,
                         uint8_t stateCount,
                         uint8_t inputCount)
{
    memset(leso, 0, sizeof(*leso));
    leso->stateCount = stateCount;
    leso->inputCount = inputCount;
}

void Algorithm_LESO_Seed(algorithm_leso_t *leso, const float *measurement)
{
    uint32_t index;

    memset(leso->estimate, 0, sizeof(leso->estimate));
    memset(leso->innovation, 0, sizeof(leso->innovation));
    for (index = 0U; index < leso->stateCount; index++)
    {
        leso->estimate[index] = measurement[index];
    }
}

void Algorithm_LESO_Update(algorithm_leso_t *leso,
                           const float *Ad,
                           const float *Bd,
                           const float *L,
                           const float *measurement,
                           const float *input,
                           const float *disturbanceLimit)
{
    float next[ALGORITHM_LESO_MAX_TOTAL_COUNT];
    uint32_t stateCount = leso->stateCount;
    uint32_t inputCount = leso->inputCount;
    uint32_t row;
    uint32_t column;

    /* C_e = [I 0]，残差就是测量减估计的前 stateCount 项。 */
    for (row = 0U; row < stateCount; row++)
    {
        leso->innovation[row] = measurement[row] - leso->estimate[row];
    }

    /* 原状态行：Ad*x_hat + Bd*(d_hat + u) + L*e。 */
    for (row = 0U; row < stateCount; row++)
    {
        float sum = 0.0f;

        for (column = 0U; column < stateCount; column++)
        {
            sum += Ad[(row * stateCount) + column] * leso->estimate[column];
        }
        for (column = 0U; column < inputCount; column++)
        {
            /* 扰动与输入同通道，合并后只过一次 Bd。 */
            sum += Bd[(row * inputCount) + column] *
                   (leso->estimate[stateCount + column] + input[column]);
        }
        for (column = 0U; column < stateCount; column++)
        {
            sum += L[(row * stateCount) + column] * leso->innovation[column];
        }
        next[row] = sum;
    }

    /* 扰动行：A_e 下半块是 [0 I]、B_e 下半块为0，只剩自身加修正。 */
    for (row = 0U; row < inputCount; row++)
    {
        uint32_t index = stateCount + row;
        float sum = leso->estimate[index];

        for (column = 0U; column < stateCount; column++)
        {
            sum += L[(index * stateCount) + column] * leso->innovation[column];
        }
        /* 限幅必须夹在递推内部，否则扰动状态会持续累积。 */
        if ((disturbanceLimit != NULL) && (disturbanceLimit[row] > 0.0f))
        {
            sum = Algorithm_LimitSymmetric(sum, disturbanceLimit[row]);
        }
        next[index] = sum;
    }

    memcpy(leso->estimate, next, sizeof(float) * (stateCount + inputCount));
}
