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
#include "pybricks/experimental/pursuit.h"
#include "pybricks/experimental/platform_math.h"
#include <pbio/imu.h>

// pbio_imu_get_heading() returns DEGREES, clockwise-positive (see lib/pbio/src/imu.c,
// which negates the mapped z axis "for positive-clockwise convention").
// This odometry frame is RADIANS, counter-clockwise-positive, matching the encoder
// math on old-build. Convert and flip at the point of reading so every other use of
// the value is already in native units.
// If heading tracks backwards on your robot, remove the leading minus sign.
#define PB_IMU_HEADING_TO_RAD   (-0.0174532925f)

pb_odom_state_t odom_state = {0};

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

    if (current_time_ms - odom_state.last_time_ms < odom_state.mstowait) {
        return;
    }
    odom_state.last_time_ms = current_time_ms;
    odom_state.vm_loop_counter++;

    int32_t cur_l, cur_r, unused_rate;
    pbio_servo_get_state_user(odom_state.left_servo, &cur_l, &unused_rate);
    pbio_servo_get_state_user(odom_state.right_servo, &cur_r, &unused_rate);

    int32_t delta_l = cur_l - odom_state.last_left_angle;
    int32_t delta_r = cur_r - odom_state.last_right_angle;
    odom_state.last_left_angle = cur_l;
    odom_state.last_right_angle = cur_r;

    // Already radians, counter-clockwise-positive.
    float current_heading = pb_odom_read_heading();
    float delta_h = current_heading - odom_state.last_imu_heading;
    odom_state.last_imu_heading = current_heading;

    // Safety net in case the heading source ever wraps. At this tick rate a real
    // half-turn between two samples is not physically possible, so this can only
    // ever catch a wrap, never clip a genuine rotation.
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
}

mp_obj_t experimental_start_odometry(size_t n_args, const mp_obj_t *args) {
    // Park the background hooks while the state is half-written. Pursuit must go
    // down too: it reads odom_state and its old path no longer relates to the
    // coordinate frame we are about to install.
    odom_state.running = false;
    pursuit_state.running = false;

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
    odom_state.mstowait = 1000 / (uint32_t)fps;
    if (odom_state.mstowait == 0) {
        odom_state.mstowait = 1;
    }

    int32_t unused;
    pbio_servo_get_state_user(odom_state.left_servo, (int32_t *)&odom_state.last_left_angle, &unused);
    pbio_servo_get_state_user(odom_state.right_servo, (int32_t *)&odom_state.last_right_angle, &unused);

    // Baseline must be stored in the same units the loop compares against.
    odom_state.last_imu_heading = pb_odom_read_heading();

    odom_state.last_time_ms = mp_hal_ticks_ms();
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
    // Pursuit cannot run without odometry, so take it down first rather than
    // leaving it steering off a frozen pose.
    if (pursuit_state.running) {
        experimental_stop_pursuit();
    }
    odom_state.running = false;
    return mp_const_none;
}

mp_obj_t experimental_get_fps(void) {
    return mp_obj_new_int_from_uint(odom_state.current_fps);
}

#endif // PYBRICKS_PY_EXPERIMENTAL
