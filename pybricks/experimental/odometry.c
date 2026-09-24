// SPDX-License-Identifier: MIT
#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/runtime.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <pybricks/common.h>
#include <pbio/servo.h>
#include <pbio/control.h>
#include "pybricks/experimental/odometry.h"
#include "pybricks/experimental/pursuit.h"
#include "pybricks/experimental/platform_math.h"
#include <pbio/imu.h>

#define PB_IMU_HEADING_TO_RAD   (-0.0174532925f)

pb_odom_state_t odom_state = {0};

static uint32_t last_tick_us;
static uint32_t interval_us;

static float pb_odom_read_heading(void) {
    return pbio_imu_get_heading(PBIO_IMU_HEADING_TYPE_1D) * PB_IMU_HEADING_TO_RAD;
}

void pb_background_odometry_update(void) {
    if (!odom_state.running) {
        return;
    }
    uint32_t current_time_ms = mp_hal_ticks_ms();

    if (current_time_ms - odom_state.last_fps_time_ms >= 1000) {
        odom_state.current_fps = odom_state.vm_loop_counter;
        odom_state.vm_loop_counter = 0;
        odom_state.last_fps_time_ms = current_time_ms;
    }

    if (!odom_state.left_servo || !odom_state.right_servo) {
        return;
    }

    uint32_t now_us = (uint32_t)mp_hal_ticks_us();
    uint32_t elapsed_us = now_us - last_tick_us;
    if (elapsed_us < interval_us) {
        return;
    }

    int32_t cur_l, cur_r, unused_rate;
    if (pbio_servo_get_state_user(odom_state.left_servo, &cur_l, &unused_rate) != PBIO_SUCCESS ||
        pbio_servo_get_state_user(odom_state.right_servo, &cur_r, &unused_rate) != PBIO_SUCCESS) {
        if (pursuit_state.running) {
            experimental_stop_pursuit();
        }
        return;
    }
    last_tick_us = now_us;
    odom_state.last_time_ms = current_time_ms;
    odom_state.vm_loop_counter++;

    int32_t delta_l = cur_l - odom_state.last_left_angle;
    int32_t delta_r = cur_r - odom_state.last_right_angle;
    odom_state.last_left_angle = cur_l;
    odom_state.last_right_angle = cur_r;

    // ccw positive rads
    float current_heading = pb_odom_read_heading();
    float delta_h = current_heading - odom_state.last_imu_heading;
    odom_state.last_imu_heading = current_heading;

    while (delta_h > 3.14159f) {
        delta_h -= 6.28318f;
    }
    while (delta_h < -3.14159f) {
        delta_h += 6.28318f;
    }

    float avg_heading = odom_state.global_h + (delta_h * 0.5f);
    odom_state.global_h += delta_h;

    while (odom_state.global_h > 3.14159f) {
        odom_state.global_h -= 6.28318f;
    }
    while (odom_state.global_h < -3.14159f) {
        odom_state.global_h += 6.28318f;
    }

    if (delta_l != 0 || delta_r != 0) {
        float dL = (float)delta_l * odom_state.deg_to_mm;
        float dR = (float)delta_r * odom_state.deg_to_mm;
        float dD = (dR + dL) * 0.5f;

        odom_state.global_x += dD * pb_fast_cos(avg_heading);
        odom_state.global_y += dD * pb_fast_sin(avg_heading);
    }

    pb_pursuit_step((float)elapsed_us * 1e-6f);
}

mp_obj_t experimental_start_odometry(size_t n_args, const mp_obj_t *args) {
    if (pursuit_state.running) {
        experimental_stop_pursuit();
    }
    odom_state.running = false;

    // pb_type_motor_get_servo() resolves subclassed motors correctly and raises a
    // proper Python error on a wrong type, unlike a raw struct cast.
    odom_state.left_servo = pb_type_motor_get_servo(args[0]);
    odom_state.right_servo = pb_type_motor_get_servo(args[1]);

    float track = mp_obj_get_float(args[3]);
    if (track <= 0.0f) {
        mp_raise_ValueError(MP_ERROR_TEXT("track width must be positive"));
    }

    odom_state.deg_to_mm = mp_obj_get_float(args[2]);
    odom_state.inv_track = 1.0f / track;
    odom_state.global_x = mp_obj_get_float(args[4]);
    odom_state.global_y = mp_obj_get_float(args[5]);
    odom_state.global_h = mp_obj_get_float(args[6]);

    int fps = mp_obj_get_int(args[7]);
    if (fps < 1) {
        mp_raise_ValueError(MP_ERROR_TEXT("fps must be at least 1"));
    }
    if (fps > 1000) {
        fps = 1000;
    }
    interval_us = 1000000u / (uint32_t)fps;
    odom_state.mstowait = 1000 / (uint32_t)fps;
    if (odom_state.mstowait == 0) {
        odom_state.mstowait = 1;
    }

    int32_t base_l, base_r, unused;
    if (pbio_servo_get_state_user(odom_state.left_servo, &base_l, &unused) != PBIO_SUCCESS ||
        pbio_servo_get_state_user(odom_state.right_servo, &base_r, &unused) != PBIO_SUCCESS) {
        mp_raise_ValueError(MP_ERROR_TEXT("could not read motor angles"));
    }
    odom_state.last_left_angle = base_l;
    odom_state.last_right_angle = base_r;
    odom_state.last_imu_heading = pb_odom_read_heading();

    odom_state.last_time_ms = mp_hal_ticks_ms();
    last_tick_us = (uint32_t)mp_hal_ticks_us();
    odom_state.last_fps_time_ms = odom_state.last_time_ms;
    odom_state.vm_loop_counter = 0;
    odom_state.current_fps = 0;

    odom_state.running = true;
    return mp_const_none;
}

mp_obj_t experimental_get_odometry(void) {
    mp_obj_t tuple[3] = {
        mp_obj_new_float_from_f(odom_state.global_x),
        mp_obj_new_float_from_f(odom_state.global_y),
        mp_obj_new_float_from_f(odom_state.global_h)
    };
    return mp_obj_new_tuple(3, tuple);
}

mp_obj_t experimental_stop_odometry(void) {
    // Pursuit cannot run without odometry
    if (pursuit_state.running) {
        experimental_stop_pursuit();
    }
    odom_state.running = false;
    return mp_const_none;
}

mp_obj_t experimental_get_fps(void) {
    return mp_obj_new_int_from_uint(odom_state.current_fps);
}

void pb_experimental_reset(void) {
    pb_pursuit_reset();
    memset(&odom_state, 0, sizeof(odom_state));
    last_tick_us = 0;
    interval_us = 0;
}

#endif // PYBRICKS_PY_EXPERIMENTAL
