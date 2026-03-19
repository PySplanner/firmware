#ifndef PYBRICKS_PLATFORM_MATH_H
#define PYBRICKS_PLATFORM_MATH_H

#include <math.h>
#include <stdint.h> // Required for uint32_t

static inline float pb_fast_sin(float theta) {
    float x = theta;

    // 1. Range Reduction to [-PI, PI]
    #if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    // Spike Prime (Cortex-M4F) optimized wrap
    float x_wrap = theta * 0.159154943f;
    x = theta - (float)((int)(x_wrap + (x_wrap > 0.0f ? 0.5f : -0.5f))) * 6.2831853f;
    #else
    //loser ev3 wrap
    while (x > 3.14159265f) {
        x -= 6.28318531f;
    }
    while (x < -3.14159265f) {
        x += 6.28318531f;
    }
    #endif

    // 2. normalizing
    if (x > 1.5707963f) {
        x = 3.1415926f - x;
    } else if (x < -1.5707963f) {
        x = -3.1415926f - x;
    }

    //minimax approximation
    float x2 = x * x;
    return x * (0.99999906f + x2 * (-0.16665554f + x2 * (0.00831190f + x2 * -0.00018488f))); //horners method to save cycles
    
}

static inline float pb_fast_cos(float theta) {
    return pb_fast_sin(theta + 1.57079633f);
}

// ---------------------------------------------------------
// skrauzys nightmare ^1.2
// ---------------------------------------------------------

// use 1 newton iteration without division, using x^1.2 = x^2 * (x^-0.2)^4
static inline float pb_fast_pow_1_2_ultra(float x) {
    // protect against zero
    if (x > -0.0001f && x < 0.0001f) return 0.0f;

    uint32_t i = *(uint32_t*)&x;              // evil floating point bit level hacking
    i &= 0x7FFFFFFF;                          // absolute value

    uint32_t iy = 1278004968 - (i / 5);       // what the fuck?
    float y = *(float*)&iy;

    float y2 = y * y;
    float y4 = y2 * y2;
    float y5 = y4 * y;
    float y_new = y * (1.2f - 0.2f * x * y5); // 1st iteration
    // y_new = y_new * ( ... );               // 2nd iteration, this can be removed

    //apply x^1.2 = x^2 * (x^-0.2)^4
    float yn2 = y_new * y_new;
    float yn4 = yn2 * yn2;
    float x2 = x * x;

    return x2 * yn4;
}

//skrauzys nightmare derivative
static inline float pb_fast_s_curve_accel(float x, float a, float b, float c) {
    float x2 = x * x;

    float u = 3.0f * a * x2 + 2.0f * b * x + c;

    float num = 6.0f * a * x + 2.0f * b;

    float base = num / ((u * u) + 1.0f);

    
    return 560.0f * pb_fast_pow_1_2_ultra(base); //use the legendary hack
}

#endif // PYBRICKS_PLATFORM_MATH_H