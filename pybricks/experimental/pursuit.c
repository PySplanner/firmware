// SPDX-License-Identifier: MIT
#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/runtime.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include <pbio/servo.h>
#include <pbio/control.h>
#include "pybricks/experimental/odometry.h"
#include "pybricks/experimental/pursuit.h"
#include "pybricks/experimental/platform_math.h"

pb_pursuit_state_t pursuit_state = {0};

static float evaluate_x(float tau, uint8_t derivative) {
    uint16_t spline_count;
    if (tau != pursuit_state.total_splines) {
        spline_count = (uint16_t)floorf(tau);
    } else {
        spline_count = pursuit_state.total_splines - 1;
    }

    float t = tau - (float)spline_count;
    float ax = pursuit_state.spline_coefficients[spline_count][0];
    float bx = pursuit_state.spline_coefficients[spline_count][1];
    float cx = pursuit_state.spline_coefficients[spline_count][2];
    float dx = pursuit_state.spline_coefficients[spline_count][3];

    if (derivative == 0) {
        return ax * t * t * t + bx * t * t + cx * t + dx;
    }
    if (derivative == 1) {
        return 3.0f * ax * t * t + 2.0f * bx * t + cx;
    }
    if (derivative == 2) {
        return 6.0f * ax * t + 2.0f * bx;
    }
    return 0.0f;
}

static float evaluate_y(float tau, uint8_t derivative) {
    uint16_t spline_count;
    if (tau != pursuit_state.total_splines) {
        spline_count = (uint16_t)floorf(tau);
    } else {
        spline_count = pursuit_state.total_splines - 1;
    }

    float t = tau - (float)spline_count;
    float ay = pursuit_state.spline_coefficients[spline_count][4];
    float by = pursuit_state.spline_coefficients[spline_count][5];
    float cy = pursuit_state.spline_coefficients[spline_count][6];
    float dy = pursuit_state.spline_coefficients[spline_count][7];

    if (derivative == 0) {
        return ay * t * t * t + by * t * t + cy * t + dy;
    }
    if (derivative == 1) {
        return 3.0f * ay * t * t + 2.0f * by * t + cy;
    }
    if (derivative == 2) {
        return 6.0f * ay * t + 2.0f * by;
    }
    return 0.0f;
}

static void target_point_approximation(void) {
    for (uint8_t i = 0; i < pursuit_state.total_newton_iterations; i++) {

        float last_t_lookahead = pursuit_state.t_lookahead;

        float dx = evaluate_x(last_t_lookahead, 0) - odom_state.global_x;
        float dy = evaluate_y(last_t_lookahead, 0) - odom_state.global_y;
        float num = (dx * dx) + (dy * dy) - (pursuit_state.lookahead * pursuit_state.lookahead);
        float den = 2.0f * dx * evaluate_x(last_t_lookahead, 1) + 2.0f * dy * evaluate_y(last_t_lookahead, 1);

        if (den != 0.0f) {
            pursuit_state.t_lookahead = last_t_lookahead - (num / den) / 3.0f;
        }
        if (pursuit_state.t_lookahead > (float)pursuit_state.total_splines) {
            pursuit_state.t_lookahead = 0.0f;
        }
    }
    pursuit_state.target_x = evaluate_x(pursuit_state.t_lookahead, 0);
    pursuit_state.target_y = evaluate_y(pursuit_state.t_lookahead, 0);
}

static float calculate_pure_pursuit(void) {
    float world_x_diff = pursuit_state.target_x - odom_state.global_x;
    float world_y_diff = pursuit_state.target_y - odom_state.global_y;
    float relative_y = (world_y_diff * pb_fast_cos(odom_state.global_h)) - (world_x_diff * pb_fast_sin(odom_state.global_h));
    float dist_sq = (world_x_diff * world_x_diff) + (world_y_diff * world_y_diff);

    if (relative_y > 0.001f || relative_y < -0.001f) {
        return -(dist_sq / (2.0f * relative_y));
    }
    return 0.0f;
}

static float evaluate_path_curvature(void) {
    float dx = evaluate_x(pursuit_state.t_lookahead, 1);
    float dy = evaluate_y(pursuit_state.t_lookahead, 1);
    float ddx = evaluate_x(pursuit_state.t_lookahead, 2);
    float ddy = evaluate_y(pursuit_state.t_lookahead, 2);

    float curvature = 0.0f;
    if (dx != 0.0f || dy != 0.0f) {
        float num = fabsf(dx * ddy - dy * ddx);
        float den_b = (dx * dx) + (dy * dy);
        float den = den_b * sqrtf(den_b);
        curvature = num / den;
    }

    if (curvature > pursuit_state.max_curvature) {
        curvature = pursuit_state.max_curvature;
    }
    if (curvature < pursuit_state.min_curvature) {
        curvature = pursuit_state.min_curvature;
    }
    return curvature;
}

static void execute_speed_control(float turning_radius, float path_curvature) {
    float local_max_speed = pursuit_state.min_speed +
        (path_curvature - pursuit_state.max_curvature) *
        (pursuit_state.max_speed - pursuit_state.min_speed) /
        (pursuit_state.min_curvature - pursuit_state.max_curvature);

    float local_base_speed = local_max_speed * pursuit_state.base_speed_percentage;
    float right_target = 0.0f, left_target = 0.0f;

    if (turning_radius != 0.0f) {
        float track_half = (1.0f / odom_state.inv_track) / 2.0f;
        right_target = local_base_speed * (turning_radius + track_half) / turning_radius;
        left_target = local_base_speed * (turning_radius - track_half) / turning_radius;
    }

    float time_passed = (float)odom_state.mstowait / 1000.0f;
    float right_accel = right_target - pursuit_state.right_motor_speed;
    float left_accel = left_target - pursuit_state.left_motor_speed;
    float max_step = pursuit_state.max_per_motor_acceleration * time_passed;

    if (fabsf(right_accel) > 0.0f || fabsf(left_accel) > 0.0f) {
        if (fabsf(right_accel) >= fabsf(left_accel)) {
            float accel_ratio = left_accel / right_accel;
            if (fabsf(right_accel) > max_step) {
                right_accel = max_step * (right_accel > 0 ? 1.0f : -1.0f);
            }
            left_accel = right_accel * accel_ratio;
        } else {
            float accel_ratio = right_accel / left_accel;
            if (fabsf(left_accel) > max_step) {
                left_accel = max_step * (left_accel > 0 ? 1.0f : -1.0f);
            }
            right_accel = left_accel * accel_ratio;
        }
    }

    pursuit_state.right_motor_speed += right_accel;
    pursuit_state.left_motor_speed += left_accel;

    float current_robot_speed = (pursuit_state.right_motor_speed + pursuit_state.left_motor_speed) / 2.0f;
    if (current_robot_speed > pursuit_state.max_speed) {
        current_robot_speed = pursuit_state.max_speed;
    }
    if (current_robot_speed < pursuit_state.min_speed) {
        current_robot_speed = pursuit_state.min_speed;
    }

    pursuit_state.lookahead = pursuit_state.min_lookahead +
        (current_robot_speed - pursuit_state.min_speed) *
        (pursuit_state.max_lookahead - pursuit_state.min_lookahead) /
        (pursuit_state.max_speed - pursuit_state.min_speed);

    pbio_servo_run_forever(odom_state.left_servo, (int32_t)pursuit_state.left_motor_speed);
    pbio_servo_run_forever(odom_state.right_servo, (int32_t)pursuit_state.right_motor_speed);
}

void pb_background_pursuit_update(void) {
    if (!pursuit_state.running || !odom_state.left_servo || !odom_state.right_servo) {
        return;
    }

    uint32_t now = mp_hal_ticks_ms();
    if (now - pursuit_state.last_time_ms < odom_state.mstowait) {
        return;
    }
    pursuit_state.last_time_ms = now;

    if (pursuit_state.t_lookahead >= (float)pursuit_state.total_splines) {
        pursuit_state.running = false;
        pbio_servo_stop(odom_state.left_servo, PBIO_CONTROL_ON_COMPLETION_BRAKE);
        pbio_servo_stop(odom_state.right_servo, PBIO_CONTROL_ON_COMPLETION_BRAKE);
        return;
    }

    target_point_approximation();
    float turning_radius = calculate_pure_pursuit();
    float path_curvature = evaluate_path_curvature();
    execute_speed_control(turning_radius, path_curvature);
}

mp_obj_t experimental_start_pursuit(size_t n_args, const mp_obj_t *args) {
    size_t num_splines;
    mp_obj_t *splines_arr;
    mp_obj_get_array(args[0], &num_splines, &splines_arr);
    pursuit_state.total_splines = (uint16_t)num_splines;
    pursuit_state.spline_coefficients = (float (*)[8])m_new(float, 8 * num_splines);

    for (size_t i = 0; i < num_splines; i++) {
        size_t num_coeffs;
        mp_obj_t *coeffs_arr;
        mp_obj_get_array(splines_arr[i], &num_coeffs, &coeffs_arr);
        for (size_t j = 0; j < 8; j++) {
            pursuit_state.spline_coefficients[i][j] = mp_obj_get_float(coeffs_arr[j]);
        }
    }

    size_t db_len;
    mp_obj_t *db_arr;
    mp_obj_get_array(args[1], &db_len, &db_arr);
    pursuit_state.max_speed = mp_obj_get_float(db_arr[2]);
    pursuit_state.min_speed = mp_obj_get_float(db_arr[3]);
    pursuit_state.base_speed_percentage = mp_obj_get_float(db_arr[4]);
    pursuit_state.max_per_motor_acceleration = mp_obj_get_float(db_arr[5]);

    size_t tun_len;
    mp_obj_t *tun_arr;
    mp_obj_get_array(args[2], &tun_len, &tun_arr);
    pursuit_state.min_curvature = 1.0f / mp_obj_get_float(tun_arr[0]);
    pursuit_state.max_curvature = 1.0f / mp_obj_get_float(tun_arr[1]);
    pursuit_state.min_lookahead = mp_obj_get_float(tun_arr[2]);
    pursuit_state.max_lookahead = mp_obj_get_float(tun_arr[3]);
    pursuit_state.total_newton_iterations = (uint8_t)mp_obj_get_int(tun_arr[4]);

    pursuit_state.t_lookahead = 0.5f;
    pursuit_state.lookahead = pursuit_state.min_lookahead;
    pursuit_state.left_motor_speed = 0.0f;
    pursuit_state.right_motor_speed = 0.0f;
    pursuit_state.running = true;
    return mp_const_none;
}

mp_obj_t experimental_stop_pursuit(void) {
    pursuit_state.running = false;
    if (odom_state.left_servo && odom_state.right_servo) {
        pbio_servo_stop(odom_state.left_servo, PBIO_CONTROL_ON_COMPLETION_BRAKE);
        pbio_servo_stop(odom_state.right_servo, PBIO_CONTROL_ON_COMPLETION_BRAKE);
    }
    return mp_const_none;
}

#endif // PYBRICKS_PY_EXPERIMENTAL
