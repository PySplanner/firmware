#ifndef PYBRICKS_PLATFORM_MATH_H
#define PYBRICKS_PLATFORM_MATH_H

#include <math.h>

// ---------------------------------------------------------
// SPIKE PRIME (Cortex-M4F) - Hardware FPU Minimax
// ---------------------------------------------------------
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)

static inline float pb_fast_sin(float theta) {
    float x_wrap = theta * 0.159154943f;
    float x = theta - (float)((int)(x_wrap + (x_wrap > 0.0f ? 0.5f : -0.5f))) * 6.2831853f;

    if (x > 1.5707963f) {
        x = 3.1415926f - x;
    } else if (x < -1.5707963f) {
        x = -3.1415926f - x;
    }

    float x2 = x * x;
    return x * (0.99999906f + x2 * (-0.16665554f + x2 * (0.00831190f + x2 * -0.00018488f)));
}

static inline float pb_fast_cos(float theta) {
    return pb_fast_sin(theta + 1.57079633f);
}

// ---------------------------------------------------------
// EV3 (ARM9) - glibc `<math.h>` optimized fallbacks
// ---------------------------------------------------------
#else

static inline float pb_fast_sin(float theta) {
    while (theta > 3.14159265f) theta -= 6.28318531f;
    while (theta < -3.14159265f) theta += 6.28318531f;
    return sinf(theta);
}

static inline float pb_fast_cos(float theta) {
    while (theta > 3.14159265f) theta -= 6.28318531f;
    while (theta < -3.14159265f) theta += 6.28318531f;
    return cosf(theta);
}

#endif
#endif // PYBRICKS_PLATFORM_MATH_H