#include "LESO.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define TEST_TOLERANCE 1.0e-5f

/*
 * 参考模型：双积分器 x1'=x2、x2'=u，Ts=0.01 做ZOH离散。
 * 扩张一维扰动后极点配到0.60/0.70/0.90，L由scipy place_poles离线求出。
 */
#define TEST_TS 0.01f

static const float testAd[2 * 2] = {
    1.0f, TEST_TS,
    0.0f, 1.0f,
};

static const float testBd[2 * 1] = {
    TEST_TS * TEST_TS * 0.5f,
    TEST_TS,
};

static const float testL[3 * 2] = {
    0.300005099f, 0.011004205f,
    0.001015513f, 0.499994901f,
    0.020395283f, 3.999898027f,
};

static void assertNear(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

/** @brief 用参考模型推进一步真实被控对象，扰动与输入同通道注入。 */
static void plantStep(float state[2], float input, float disturbance)
{
    float total = input + disturbance;
    float next0 = (testAd[0] * state[0]) + (testAd[1] * state[1]) +
                  (testBd[0] * total);
    float next1 = (testAd[2] * state[0]) + (testAd[3] * state[1]) +
                  (testBd[1] * total);

    state[0] = next0;
    state[1] = next1;
}

/* 零矩阵、零增益时估计必须原地不动，用于暴露行序或维度写反。 */
static void testIdentityHold(void)
{
    algorithm_leso_t leso;
    const float identityAd[2 * 2] = {1.0f, 0.0f, 0.0f, 1.0f};
    const float zeroBd[2 * 1] = {0.0f, 0.0f};
    const float zeroL[3 * 2] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const float measurement[2] = {7.0f, -3.0f};
    const float input[1] = {5.0f};
    uint32_t step;

    Algorithm_LESO_Init(&leso, 2U, 1U);
    Algorithm_LESO_Seed(&leso, measurement);
    assertNear(leso.estimate[0], 7.0f, TEST_TOLERANCE);
    assertNear(leso.estimate[1], -3.0f, TEST_TOLERANCE);
    assertNear(leso.estimate[2], 0.0f, TEST_TOLERANCE);

    for (step = 0U; step < 100U; step++)
    {
        Algorithm_LESO_Update(&leso, identityAd, zeroBd, zeroL,
                              measurement, input, NULL);
    }
    assertNear(leso.estimate[0], 7.0f, TEST_TOLERANCE);
    assertNear(leso.estimate[1], -3.0f, TEST_TOLERANCE);
    assertNear(leso.estimate[2], 0.0f, TEST_TOLERANCE);
    /* 残差是测量减估计，估计没动则残差恒为0。 */
    assertNear(leso.innovation[0], 0.0f, TEST_TOLERANCE);
    assertNear(leso.innovation[1], 0.0f, TEST_TOLERANCE);
}

/* LESO的核心性质：恒定输入扰动必须被扰动状态收敛估出。 */
static void testDisturbanceConverges(void)
{
    algorithm_leso_t leso;
    float plant[2] = {0.0f, 0.0f};
    const float disturbance = 0.5f;
    const float input[1] = {0.0f};
    uint32_t step;

    Algorithm_LESO_Init(&leso, 2U, 1U);
    for (step = 0U; step < 500U; step++)
    {
        float measurement[2] = {plant[0], plant[1]};

        Algorithm_LESO_Update(&leso, testAd, testBd, testL,
                              measurement, input, NULL);
        plantStep(plant, input[0], disturbance);
    }

    assertNear(leso.estimate[2], disturbance, 1.0e-4f);
    assertNear(leso.innovation[0], 0.0f, 1.0e-5f);
    assertNear(leso.innovation[1], 0.0f, 1.0e-5f);
}

/* 输入自身也走同一通道，估出的扰动应仍然只等于真实扰动。 */
static void testInputIsNotCountedAsDisturbance(void)
{
    algorithm_leso_t leso;
    float plant[2] = {0.0f, 0.0f};
    const float disturbance = -0.25f;
    const float input[1] = {1.5f};
    uint32_t step;

    Algorithm_LESO_Init(&leso, 2U, 1U);
    for (step = 0U; step < 500U; step++)
    {
        float measurement[2] = {plant[0], plant[1]};

        Algorithm_LESO_Update(&leso, testAd, testBd, testL,
                              measurement, input, NULL);
        plantStep(plant, input[0], disturbance);
    }

    assertNear(leso.estimate[2], disturbance, 1.0e-4f);
}

/* 扰动限幅必须夹在递推内部，否则扰动状态会一路windup。 */
static void testDisturbanceLimit(void)
{
    algorithm_leso_t leso;
    float plant[2] = {0.0f, 0.0f};
    const float disturbance = 0.5f;
    const float input[1] = {0.0f};
    const float limit[1] = {0.1f};
    uint32_t step;

    Algorithm_LESO_Init(&leso, 2U, 1U);
    for (step = 0U; step < 500U; step++)
    {
        float measurement[2] = {plant[0], plant[1]};

        Algorithm_LESO_Update(&leso, testAd, testBd, testL,
                              measurement, input, limit);
        plantStep(plant, input[0], disturbance);
        assert(fabsf(leso.estimate[2]) <= limit[0] + TEST_TOLERANCE);
    }

    /* 限幅为0表示该通道不限幅，与配置表里0的含义一致。 */
    const float noLimit[1] = {0.0f};

    Algorithm_LESO_Init(&leso, 2U, 1U);
    plant[0] = 0.0f;
    plant[1] = 0.0f;
    for (step = 0U; step < 500U; step++)
    {
        float measurement[2] = {plant[0], plant[1]};

        Algorithm_LESO_Update(&leso, testAd, testBd, testL,
                              measurement, input, noLimit);
        plantStep(plant, input[0], disturbance);
    }
    assertNear(leso.estimate[2], disturbance, 1.0e-4f);
}

/* Seed用当前测量重建初值并清零扰动，供故障恢复后重新起观测。 */
static void testSeedClearsDisturbance(void)
{
    algorithm_leso_t leso;
    float plant[2] = {0.0f, 0.0f};
    const float input[1] = {0.0f};
    const float seed[2] = {1.0f, 2.0f};
    uint32_t step;

    Algorithm_LESO_Init(&leso, 2U, 1U);
    for (step = 0U; step < 300U; step++)
    {
        float measurement[2] = {plant[0], plant[1]};

        Algorithm_LESO_Update(&leso, testAd, testBd, testL,
                              measurement, input, NULL);
        plantStep(plant, input[0], 0.5f);
    }
    assert(fabsf(leso.estimate[2]) > 0.1f);

    Algorithm_LESO_Seed(&leso, seed);
    assertNear(leso.estimate[0], 1.0f, TEST_TOLERANCE);
    assertNear(leso.estimate[1], 2.0f, TEST_TOLERANCE);
    assertNear(leso.estimate[2], 0.0f, TEST_TOLERANCE);
}

int main(void)
{
    testIdentityHold();
    testDisturbanceConverges();
    testInputIsNotCountedAsDisturbance();
    testDisturbanceLimit();
    testSeedClearsDisturbance();
    return 0;
}
