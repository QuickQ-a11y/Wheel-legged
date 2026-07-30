#include "Angle.h"

#include <math.h>

#define ALGORITHM_PI_F 3.14159265358979323846f
#define ALGORITHM_TWO_PI_F (2.0f * ALGORITHM_PI_F)

float Algorithm_AngleNormalizeRad(float angle_rad)
{
    float normalized_rad = fmodf(angle_rad + ALGORITHM_PI_F,
                                 ALGORITHM_TWO_PI_F);

    if (normalized_rad < 0.0f)
    {
        normalized_rad += ALGORITHM_TWO_PI_F;
    }
    return normalized_rad - ALGORITHM_PI_F;
}

float Algorithm_AngleNearestEquivalentRad(float angle_rad,
                                          float reference_rad)
{
    return reference_rad +
           Algorithm_AngleNormalizeRad(angle_rad - reference_rad);
}

float Algorithm_AngleUnwrapRad(float previous_wrapped_rad,
                               float previous_total_rad,
                               float current_wrapped_rad)
{
    return previous_total_rad +
           Algorithm_AngleNormalizeRad(current_wrapped_rad -
                                       previous_wrapped_rad);
}
