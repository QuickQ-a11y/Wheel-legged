#include "Angle.h"

#include <assert.h>
#include <math.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_TOLERANCE 1.0e-5f

static void assertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) <= TEST_TOLERANCE);
}

static void testNormalize(void)
{
    assertNear(Algorithm_AngleNormalizeRad(0.0f), 0.0f);
    assertNear(Algorithm_AngleNormalizeRad(TEST_PI), -TEST_PI);
    assertNear(Algorithm_AngleNormalizeRad(-TEST_PI), -TEST_PI);
    assertNear(Algorithm_AngleNormalizeRad(3.0f * TEST_PI), -TEST_PI);
    assertNear(Algorithm_AngleNormalizeRad(-3.0f * TEST_PI), -TEST_PI);
}

static void testNearestEquivalent(void)
{
    assertNear(Algorithm_AngleNearestEquivalentRad(-TEST_PI + 0.1f,
                                                    TEST_PI + 0.05f),
                TEST_PI + 0.1f);
    assertNear(Algorithm_AngleNearestEquivalentRad(TEST_PI - 0.1f,
                                                    -TEST_PI - 0.05f),
                -TEST_PI - 0.1f);
}

static void testUnwrap(void)
{
    assertNear(Algorithm_AngleUnwrapRad(TEST_PI - 0.01f,
                                        TEST_PI - 0.01f,
                                        -TEST_PI + 0.02f),
                TEST_PI + 0.02f);
    assertNear(Algorithm_AngleUnwrapRad(-TEST_PI + 0.01f,
                                        -TEST_PI + 0.01f,
                                        TEST_PI - 0.02f),
                -TEST_PI - 0.02f);
}

int main(void)
{
    testNormalize();
    testNearestEquivalent();
    testUnwrap();
    return 0;
}
