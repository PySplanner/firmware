#ifndef PYBRICKS_PLATFORM_MATH_H
#define PYBRICKS_PLATFORM_MATH_H

#include <math.h>
#include <stdint.h>

static inline float pb_fast_sin(float theta) {
    float x = theta;

    // 1. Range Reduction to [-PI, PI]
    #if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    float x_wrap = theta * 0.159154943f;
    x = theta - (float)((int)(x_wrap + (x_wrap > 0.0f ? 0.5f : -0.5f))) * 6.2831853f;
    #else
    while (x > 3.14159265f) {
        x -= 6.28318531f;
    }
    while (x < -3.14159265f) {
        x += 6.28318531f;
    }
    #endif

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

// ----------------------------------------------------------------------------
// Quake III Style - Fast Inverse Square Root (Legalized for modern compilers)
// ----------------------------------------------------------------------------
static inline float pb_fast_pow_1_2_ultra(float x) {
    if (x > -0.0001f && x < 0.0001f) {
        return 0.0f;
    }

    // We use a union to tell the compiler: "This memory is both a float AND a long"
    // This bypasses strict-aliasing errors while keeping the bit-hacking intact.
    union {
        float f;
        uint32_t i;
    } conv; // evil floating point bit level hacking

    float x2;
    const float threehalfs = 1.5f;

    x2 = x * 0.5f;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);                  // what the fuck?

    float y = conv.f;
    y = y * (threehalfs - (x2 * y * y));        // 1st iteration
    // y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, can be removed

    float yn2 = y * y;
    float yn4 = yn2 * yn2;
    return (x * x) * yn4;
}

static inline float pb_fast_s_curve_accel(float x, float a, float b, float c) {
    float x2 = x * x;
    float u = 3.0f * a * x2 + 2.0f * b * x + c;
    float num = 6.0f * a * x + 2.0f * b;
    float base = num / ((u * u) + 1.0f);

    return 560.0f * pb_fast_pow_1_2_ultra(base);
}

#endif // PYBRICKS_PLATFORM_MATH_H
