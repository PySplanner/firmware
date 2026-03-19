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

// Hardware acceleration for the Spike Prime
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

ACCEL_RAM static float fast_cos_internal(float theta) {
    return fast_sin_internal(theta + 1.57079633f);
}

// ---------------------------------------------------------
// BACKGROUND ODOMETRY STATE MACHINE
// ---------------------------------------------------------
volatile bool odom_running = false;
volatile uint32_t last_odom_time_ms = 0;

volatile float global_x = 0.0f;
volatile float global_y = 0.0f;
volatile float global_h = 0.0f;

volatile int32_t last_left_angle = 0;
volatile int32_t last_right_angle = 0;

float odom_deg_to_mm = 1.0f;
float odom_inv_track = 1.0f;

pbio_servo_t *left_servo_ptr = NULL;
pbio_servo_t *right_servo_ptr = NULL;

// This is the hook that will run continuously in the OS background
void pb_background_odometry_update(void) {
    if (!odom_running) return;

    // Throttle to 200Hz (5ms) to prevent starving the motor controllers
    uint32_t now = mp_hal_ticks_ms();
    if (now - last_odom_time_ms < 5) return;
    last_odom_time_ms = now;

    int32_t cur_l, cur_r, unused_rate;
    pbio_servo_get_state_user(left_servo_ptr, &cur_l, &unused_rate);
    pbio_servo_get_state_user(right_servo_ptr, &cur_r, &unused_rate);

    int32_t delta_l = cur_l - last_left_angle;
    int32_t delta_r = cur_r - last_right_angle;

    if (delta_l != 0 || delta_r != 0) {
        float dL = (float)delta_l * odom_deg_to_mm;
        float dR = (float)delta_r * odom_deg_to_mm;

        float dD = (dR + dL) * 0.5f;
        float dH = (dR - dL) * odom_inv_track;
        float avg_h = global_h + (dH * 0.5f);

        global_x += dD * fast_cos_internal(avg_h);
        global_y += dD * fast_sin_internal(avg_h);
        global_h += dH;

        last_left_angle = cur_l;
        last_right_angle = cur_r;
    }
}

// Python API: experimental.start_odometry(left_motor, right_motor, circ, track, x, y, h)
mp_obj_t experimental_start_odometry(size_t n_args, const mp_obj_t *args) {
    left_servo_ptr = ((pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(args[0]))->srv;
    right_servo_ptr = ((pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(args[1]))->srv;
    
    odom_deg_to_mm = mp_obj_get_float(args[2]) / 360.0f;
    odom_inv_track = 1.0f / mp_obj_get_float(args[3]);
    
    global_x = mp_obj_get_float(args[4]);
    global_y = mp_obj_get_float(args[5]);
    global_h = mp_obj_get_float(args[6]);

    int32_t unused_rate;
    pbio_servo_get_state_user(left_servo_ptr, (int32_t*)&last_left_angle, &unused_rate);
    pbio_servo_get_state_user(right_servo_ptr, (int32_t*)&last_right_angle, &unused_rate);

    odom_running = true;
    return mp_const_none;
}

// Python API: experimental.get_odometry() -> (x, y, h)
mp_obj_t experimental_get_odometry(void) {
    mp_obj_t tuple[3] = {
        mp_obj_new_float_from_f(global_x),
        mp_obj_new_float_from_f(global_y),
        mp_obj_new_float_from_f(global_h)
    };
    return mp_obj_new_tuple(3, tuple);
}

// Python API: experimental.stop_odometry()
mp_obj_t experimental_stop_odometry(void) {
    odom_running = false;
    return mp_const_none;
}

// ---------------------------------------------------------
// PURE PURSUIT (Unchanged)
// ---------------------------------------------------------
mp_obj_t get_pure_pursuit_multipliers(float tx, float ty, float rx_pos, float ry_pos, float rh_ang, float track) {
    float x_dif = tx - rx_pos;
    float y_dif = ty - ry_pos;

    float cos_h = fast_cos_internal(rh_ang);
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