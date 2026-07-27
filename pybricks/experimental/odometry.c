// SPDX-License-Identifier: MIT
#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/runtime.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include <pybricks/common.h>
#include <pbio/servo.h>
#include <pbio/control.h>
#include "pybricks/experimental/odometry.h"
#include "pybricks/experimental/platform_math.h"

#include <pbio/imu.h>

// Hardware Object structure declaration
typedef struct _pb_type_pupdevices_Motor_obj_t {
    mp_obj_base_t base;
    pbio_servo_t *srv;
} pb_type_pupdevices_Motor_obj_t;

// State Variables
static pbio_servo_t *left_servo_ptr = NULL;
static pbio_servo_t *right_servo_ptr = NULL;

volatile bool odom_running = false;
volatile uint32_t last_odom_time_ms = 0;
volatile uint32_t last_pursuit_time_ms = 0;

volatile uint32_t fps = 200;
volatile uint32_t mstowait = 5;
volatile float global_x = 0.0f, global_y = 0.0f, global_h = 0.0f;
volatile int32_t last_left_angle = 0, last_right_angle = 0;

// NEW: Store the last IMU read to calculate deltas
volatile float last_imu_heading = 0.0f;

float odom_deg_to_mm = 1.0f;
float odom_inv_track = 1.0f;

volatile bool pursuit_running = false;
volatile float p_target_speed = 0.0f;
volatile float p_lookahead = 120.0f;
volatile float sp_a = 0.0f, sp_b = 0.0f, sp_c = 0.0f, sp_d = 0.0f, sp_x_end = 0.0f;

// FPS Counter Variables
volatile uint32_t vm_loop_counter = 0;
volatile uint32_t current_fps = 0;
volatile uint32_t last_fps_time_ms = 0;

void pb_background_odometry_update(void) {
    if (!odom_running) {
        return;
    }

    uint32_t now = mp_hal_ticks_ms();

    // Every 1000ms, save the count and reset
    if (now - last_fps_time_ms >= 1000) {
        current_fps = vm_loop_counter;
        vm_loop_counter = 0;
        last_fps_time_ms = now;
    }
    // -------------------------

    if (!left_servo_ptr || !right_servo_ptr) {
        return;
    }

    // --- RATE CAP: 200 Hz (5ms) ---
    if (now - last_odom_time_ms < mstowait) {
        return;
    }
    last_odom_time_ms = now;
    // ------------------------------
    vm_loop_counter++;

    int32_t cur_l, cur_r, unused_rate;
    pbio_servo_get_state_user(left_servo_ptr, &cur_l, &unused_rate);
    pbio_servo_get_state_user(right_servo_ptr, &cur_r, &unused_rate);

    int32_t delta_l = cur_l - last_left_angle;
    int32_t delta_r = cur_r - last_right_angle;

    // Update encoder state immediately
    last_left_angle = cur_l;
    last_right_angle = cur_r;

    // 2. Calculate Heading Delta from IMU (Happens even if wheels don't move, in case robot is pushed)
    float current_imu_heading = pbio_imu_get_heading(0);
    float dH = current_imu_heading - last_imu_heading;
    last_imu_heading = current_imu_heading;

    // Protect against the IMU wrapping across the -PI/PI boundary
    while (dH > 3.14159f) {
        dH -= 6.28318f;
    }
    while (dH < -3.14159f) {
        dH += 6.28318f;
    }

    // Calculate the average heading for this tick's coordinate projection
    float avg_h = global_h + (dH * 0.5f);

    // Accumulate the global heading and wrap it safely
    global_h += dH;
    while (global_h > 3.14159f) {
        global_h -= 6.28318f;
    }
    while (global_h < -3.14159f) {
        global_h += 6.28318f;
    }

    // 3. Project X/Y coordinates if the wheels actually moved
    if (delta_l != 0 || delta_r != 0) {
        float dL = (float)delta_l * odom_deg_to_mm;
        float dR = (float)delta_r * odom_deg_to_mm;
        float dD = (dR + dL) * 0.5f;

        global_x += dD * pb_fast_cos(avg_h);
        global_y += dD * pb_fast_sin(avg_h);
    }
}

void pb_background_pursuit_update(void) {
    if (!pursuit_running || !left_servo_ptr || !right_servo_ptr) {
        return;
    }

    // --- RATE CAP: 100 Hz (10ms) ---
    uint32_t now = mp_hal_ticks_ms();
    if (now - last_pursuit_time_ms < mstowait) {
        return;
    }
    last_pursuit_time_ms = now;
    // -------------------------------

    if (global_x >= sp_x_end) {
        pursuit_running = false;
        pbio_servo_stop(left_servo_ptr, PBIO_CONTROL_ON_COMPLETION_BRAKE);
        pbio_servo_stop(right_servo_ptr, PBIO_CONTROL_ON_COMPLETION_BRAKE);
        return;
    }

    float target_x = global_x + p_lookahead;
    if (target_x > sp_x_end) {
        target_x = sp_x_end;
    }

    float target_y = (sp_a * (target_x * target_x * target_x)) +
        (sp_b * (target_x * target_x)) +
        (sp_c * target_x) + sp_d;

    float x_dif = target_x - global_x;
    float y_dif = target_y - global_y;
    float relative_y = (y_dif * pb_fast_cos(global_h)) - (x_dif * pb_fast_sin(global_h));
    float dist_sq = (x_dif * x_dif) + (y_dif * y_dif);

    float m_left = 1.0f, m_right = 1.0f;
    if (relative_y > 0.001f || relative_y < -0.001f) {
        float radius = -(dist_sq / (2.0f * relative_y));
        float track = 1.0f / odom_inv_track;
        m_right = (2.0f * radius) / ((2.0f * radius) + track);
        m_left = (2.0f * radius) / ((2.0f * radius) - track);
    }

    pbio_servo_run_forever(left_servo_ptr, (int32_t)(p_target_speed * m_left));
    pbio_servo_run_forever(right_servo_ptr, (int32_t)(p_target_speed * m_right));
}

// MicroPython Wrappers
mp_obj_t experimental_start_odometry(size_t n_args, const mp_obj_t *args) {
    left_servo_ptr = ((pb_type_pupdevices_Motor_obj_t *)MP_OBJ_TO_PTR(args[0]))->srv;
    right_servo_ptr = ((pb_type_pupdevices_Motor_obj_t *)MP_OBJ_TO_PTR(args[1]))->srv;

    // Using mm_per_deg directly as passed from Python
    odom_deg_to_mm = mp_obj_get_float(args[2]);
    odom_inv_track = 1.0f / mp_obj_get_float(args[3]);

    global_x = mp_obj_get_float(args[4]);
    global_y = mp_obj_get_float(args[5]);
    global_h = mp_obj_get_float(args[6]);
    fps = mp_obj_get_int(args[7]);

    mstowait = 1000 / fps;
    int32_t unused;
    pbio_servo_get_state_user(left_servo_ptr, (int32_t *)&last_left_angle, &unused);
    pbio_servo_get_state_user(right_servo_ptr, (int32_t *)&last_right_angle, &unused);

    // Sync the IMU baseline before the loop starts to prevent massive delta jumps
    pbio_imu_t *imu;
    if (pbio_imu_get_imu(&imu) == PBIO_SUCCESS) {
        last_imu_heading = pbio_imu_get_heading(imu);
    }

    odom_running = true;
    return mp_const_none;
}

mp_obj_t experimental_get_odometry(void) {
    mp_obj_t tuple[3] = {
        mp_obj_new_float_from_f(global_x),
        mp_obj_new_float_from_f(global_y),
        mp_obj_new_float_from_f(global_h)
    };
    return mp_obj_new_tuple(3, tuple);
}

mp_obj_t experimental_start_pursuit(size_t n_args, const mp_obj_t *args) {
    sp_a = mp_obj_get_float(args[0]);
    sp_b = mp_obj_get_float(args[1]);
    sp_c = mp_obj_get_float(args[2]);
    sp_d = mp_obj_get_float(args[3]);
    sp_x_end = mp_obj_get_float(args[4]);
    p_target_speed = mp_obj_get_float(args[5]);
    p_lookahead = mp_obj_get_float(args[6]);
    pursuit_running = true;
    return mp_const_none;
}

mp_obj_t experimental_stop_pursuit(void) {
    pursuit_running = false;
    if (left_servo_ptr && right_servo_ptr) {
        pbio_servo_stop(left_servo_ptr, PBIO_CONTROL_ON_COMPLETION_BRAKE);
        pbio_servo_stop(right_servo_ptr, PBIO_CONTROL_ON_COMPLETION_BRAKE);
    }
    return mp_const_none;
}

mp_obj_t experimental_stop_odometry(void) {
    odom_running = false;
    return mp_const_none;
}

mp_obj_t experimental_get_fps(void) {
    return mp_obj_new_int_from_uint(current_fps);
}

#endif // PYBRICKS_PY_EXPERIMENTAL
