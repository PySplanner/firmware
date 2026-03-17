#ifndef PYBRICKS_PLATFORM_MATH_H
#define PYBRICKS_PLATFORM_MATH_H

#include <math.h>

static inline float pb_fast_sin(float theta) {
    float x = theta;

    // 1. Range Reduction to [-PI, PI]
    #if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
        // Spike Prime (Cortex-M4F) optimized wrap
        float x_wrap = theta * 0.159154943f; 
        x = theta - (float)((int)(x_wrap + (x_wrap > 0.0f ? 0.5f : -0.5f))) * 6.2831853f; 
    #else
        // EV3 (ARM9) stable wrap
        while (x > 3.14159265f) x -= 6.28318531f;
        while (x < -3.14159265f) x += 6.28318531f;
    #endif

    // 2. Mirroring to [-PI/2, PI/2]
    if (x > 1.5707963f) x = 3.1415926f - x;
    else if (x < -1.5707963f) x = -3.1415926f - x;
    
    // 3. The Spike Minimax Polynomial
    float x2 = x * x;
    return x * (0.99999906f + x2 * (-0.16665554f + x2 * (0.00831190f + x2 * -0.00018488f)));
}

static inline float pb_fast_cos(float theta) {
    return pb_fast_sin(theta + 1.57079633f);
}

#endif // PYBRICKS_PLATFORM_MATH_H