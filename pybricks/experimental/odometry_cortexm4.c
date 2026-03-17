// SPDX-License-Identifier: MIT
#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)

#include "py/mphal.h"
#include "py/runtime.h"
#include <math.h>
#include "pybricks/experimental/odometry.h"

// Force the math functions into RAM for zero-wait-state execution
#define ACCEL_RAM __attribute__((section(".data"), noinline))

// High-speed Minimax Polynomial for Sine/Cosine
// This replaces the heavy math.h calls with raw FPU instructions
ACCEL_RAM static float fast_sin_internal(float theta) {
    float x = theta * 0.159154943f; 
    x = theta - (float)((int)(x + (x > 0.0f ? 0.5f : -0.5f))) * 6.2831853f; 
    
    if (x > 1.5707963f) x = 3.1415926f - x;
    else if (x < -1.5707963f) x = -3.1415926f - x;
    
    float x2 = x * x;
    return x * (0.99999906f + x2 * (-0.16665554f + x2 * (0.00831190f + x2 * -0.00018488f)));
}

mp_obj_t calculate_odometry(int num_iters, float wheel_circ, float axle_track, mp_obj_t right_angle_func, mp_obj_t left_angle_func) {
    
    // Pre-calculate kinematic constants
    float deg_to_mm = wheel_circ * 0.0027777778f; // (circ / 360)
    float inv_axle_track = 1.0f / axle_track;
    
    // Robot state
    float rx = 0.0f;
    float ry = 0.0f;
    float rh = 0.0f;

    // Initial sensor grab
    int32_t last_r = mp_obj_get_int(mp_call_function_0(right_angle_func));
    int32_t last_l = mp_obj_get_int(mp_call_function_0(left_angle_func));

    uint32_t start_time = mp_hal_ticks_ms();

    for (int i = 0; i < num_iters; i++) {
        // 1. Fetch encoder data (The primary Spike bottleneck)
        int32_t cur_r = mp_obj_get_int(mp_call_function_0(right_angle_func));
        int32_t cur_l = mp_obj_get_int(mp_call_function_0(left_angle_func));

        // 2. Delta math
        float dR = (float)(cur_r - last_r) * deg_to_mm;
        float dL = (float)(cur_l - last_l) * deg_to_mm;
        float dD = (dR + dL) * 0.5f; 
        float dH = (dR - dL) * inv_axle_track; 

        // 3. RK2 Midpoint Integration
        if (dD != 0.0f || dH != 0.0f) {
            float avg_h = rh + (dH * 0.5f);
            rx += dD * fast_sin_internal(avg_h + 1.5707963f); // cos
            ry += dD * fast_sin_internal(avg_h);             // sin
            rh += dH;
        }

        last_r = cur_r;
        last_l = cur_l;

        // 4. Housekeeping (Polled, not delayed)
        // Checking every 1024 iterations (bitwise & is faster than % on M4)
        if ((i & 0x3FF) == 0) {
            mp_handle_pending(true);
        }
    }

    uint32_t dur = mp_hal_ticks_ms() - start_time;
    
    // Packaging results for Python
    mp_obj_t tuple[5] = {
        mp_obj_new_float_from_f((float)dur * 0.001f), 
        mp_obj_new_int(num_iters),
        mp_obj_new_float_from_f((float)num_iters / ((float)dur * 0.001f)), 
        mp_obj_new_float_from_f(rx),
        mp_obj_new_float_from_f(ry)
    };
    return mp_obj_new_tuple(5, tuple);
}

#endif
#endif