// SPDX-License-Identifier: MIT
#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/runtime.h"
#include <math.h>

// Core Pybricks headers for bare-metal hardware access
#include <pybricks/common.h>
#include <pbio/servo.h>
#include "pybricks/experimental/odometry.h"

// THE LIFELINE: Bring in the Pybricks background task runner!
extern bool pbio_os_run_processes_once(void);

// Hardware acceleration for the Spike Prime (FPU + RAM execution)
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
#define ACCEL_RAM __attribute__((section(".data"), noinline))
#else
#define ACCEL_RAM
#endif

ACCEL_RAM static float fast_sin_internal(float theta) {
    float x = theta * 0.159154943f;
    x = theta - (float)((int)(x + (x > 0.00f ? 0.5f : -0.5f))) * 6.2831853f;

    if (x > 1.5707963f) {
        x = 3.1415926f - x;
    } else if (x < -1.5707963f) {
        x = -3.1415926f - x;
    }

    float x2 = x * x;
    return x * (0.99999906f + x2 * (-0.16665554f + x2 * (0.00831190f + x2 * -0.00018488f)));
}

mp_obj_t calculate_odometry(int num_iters, float wheel_circ, float axle_track, mp_obj_t right_motor_obj, mp_obj_t left_motor_obj) {

    float deg_to_mm = wheel_circ * 0.0027777778f; 
    float inv_axle_track = 1.0f / axle_track;

    float rx = 0.0f;
    float ry = 0.0f;
    float rh = 0.0f;

    pbio_servo_t *srv_r = ((pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(right_motor_obj))->srv;
    pbio_servo_t *srv_l = ((pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(left_motor_obj))->srv;

    int32_t last_r, last_l, unused_rate;
    pbio_servo_get_state_user(srv_r, &last_r, &unused_rate);
    pbio_servo_get_state_user(srv_l, &last_l, &unused_rate);

    uint32_t start_time = mp_hal_ticks_ms();

    for (int i = 0; i < num_iters; i++) {
        int32_t cur_r, cur_l;
        
        // 1. Instant Memory read
        pbio_servo_get_state_user(srv_r, &cur_r, &unused_rate);
        pbio_servo_get_state_user(srv_l, &cur_l, &unused_rate);

        // 2. THE BYPASS: Only run heavy FPU math if the wheels physically moved!
        int32_t delta_r = cur_r - last_r;
        int32_t delta_l = cur_l - last_l;

        if (delta_r != 0 || delta_l != 0) {
            float dR = (float)delta_r * deg_to_mm;
            float dL = (float)delta_l * deg_to_mm;
            float dD = (dR + dL) * 0.5f;
            float dH = (dR - dL) * inv_axle_track;

            float avg_h = rh + (dH * 0.5f);
            rx += dD * fast_sin_internal(avg_h + 1.5707963f); // cos
            ry += dD * fast_sin_internal(avg_h);              // sin
            rh += dH;

            last_r = cur_r;
            last_l = cur_l;
        }

        // 3. THE LIFELINE: Pump the Pybricks OS so the motors actually drive
        // (Bitwise & is a lightning-fast way to do % 256)
        if ((i & 0xFF) == 0) {
            mp_handle_pending(true);
            while (pbio_os_run_processes_once()) {}
        }
    }

    uint32_t dur = mp_hal_ticks_ms() - start_time;

    mp_obj_t tuple[5] = {
        mp_obj_new_float_from_f((float)dur * 0.001f),
        mp_obj_new_int(num_iters),
        mp_obj_new_float_from_f((float)num_iters / ((float)dur * 0.001f)),
        mp_obj_new_float_from_f(rx),
        mp_obj_new_float_from_f(ry)
    };
    return mp_obj_new_tuple(5, tuple);
}

// Optimized Pure Pursuit Logic using Dot Products
mp_obj_t get_pure_pursuit_multipliers(float tx, float ty, float rx_pos, float ry_pos, float rh_ang, float track) {
    float x_dif = tx - rx_pos;
    float y_dif = ty - ry_pos;

    float cos_h = fast_sin_internal(rh_ang + 1.5707963f);
    float sin_h = fast_sin_internal(rh_ang);

    float relative_y = (y_dif * cos_h) - (x_dif * sin_h);

    float m_left = 1.0f;
    float m_right = 1.0f;
    
    float abs_rel_y = relative_y < 0.0f ? -relative_y : relative_y;

    if (abs_rel_y > 0.001f) {
        float dist_sq = (x_dif * x_dif) + (y_dif * y_dif);
        float radius = -(dist_sq / (2.0f * relative_y));
        float two_r = 2.0f * radius;
        m_right = two_r / (two_r + track);
        m_left = two_r / (two_r - track);
    }

    mp_obj_t tuple[2] = {
        mp_obj_new_float_from_f(m_left),
        mp_obj_new_float_from_f(m_right)
    };
    return mp_obj_new_tuple(2, tuple);
}

#endif