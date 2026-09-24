// SPDX-License-Identifier: MIT
#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/runtime.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <pbio/servo.h>
#include <pbio/control.h>
#include "pybricks/experimental/odometry.h"
#include "pybricks/experimental/pursuit.h"
#include "pybricks/experimental/platform_math.h"

pb_pursuit_state_t pursuit_state = {0};

#define PB_PURSUIT_MAX_NEWTON_STEP (0.25f)
#define PB_PURSUIT_FORWARD_NUDGE   (0.05f)
#define PB_PURSUIT_STOP_TOLERANCE_MM (5.0f)
#define PB_PURSUIT_DECEL_MARGIN      (0.8f)

static bool finishing;
static float end_x, end_y;
static int32_t last_cmd_left, last_cmd_right;
static bool cmd_valid;

// Resolves tau to a spline index plus a local parameter, clamped at both ends.
// The old version cast tau straight to uint16_t, which read out of bounds for
// any negative tau Newton happened to produce.
static uint16_t spline_index(float tau) {
    if (tau <= 0.0f) {
        return 0;
    }
    uint16_t index = (uint16_t)tau;
    if (index >= pursuit_state.total_splines) {
        index = pursuit_state.total_splines - 1;
    }
    return index;
}

static float evaluate_x(float tau, uint8_t derivative) {
    uint16_t spline_count = spline_index(tau);
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
    uint16_t spline_count = spline_index(tau);
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
    float t_end = (float)pursuit_state.total_splines;
    float t_start = pursuit_state.t_lookahead;

    for (uint8_t i = 0; i < pursuit_state.total_newton_iterations; i++)
    {

        float last_t_lookahead = pursuit_state.t_lookahead;

        float dx = evaluate_x(last_t_lookahead, 0) - odom_state.global_x;
        float dy = evaluate_y(last_t_lookahead, 0) - odom_state.global_y;
        float num = (dx * dx) + (dy * dy) - (pursuit_state.lookahead * pursuit_state.lookahead);
        float den = 2.0f * dx * evaluate_x(last_t_lookahead, 1) + 2.0f * dy * evaluate_y(last_t_lookahead, 1);

        float step = 0.0f;
        if (fabsf(den) > 1e-6f) {
            step = -(num / den) / 3.0f;
        }

        if (step > PB_PURSUIT_MAX_NEWTON_STEP) {
            step = PB_PURSUIT_MAX_NEWTON_STEP;
        }

        if (step < -PB_PURSUIT_MAX_NEWTON_STEP) {
            step = -PB_PURSUIT_MAX_NEWTON_STEP;
        }

        if (num < 0.0f && step <= 0.0f) {
            step = PB_PURSUIT_FORWARD_NUDGE;
        }
        pursuit_state.t_lookahead = last_t_lookahead + step;

        // clamping to path
        if (pursuit_state.t_lookahead > t_end) {
            pursuit_state.t_lookahead = t_end;
        }
        if (pursuit_state.t_lookahead < t_start) {
            pursuit_state.t_lookahead = t_start;
        }
    }
    pursuit_state.target_x = evaluate_x(pursuit_state.t_lookahead, 0);
    pursuit_state.target_y = evaluate_y(pursuit_state.t_lookahead, 0);
}

// fixed 0.0f being interpreted as 0 speed
static float calculate_pure_pursuit(void) {
    float world_x_diff = pursuit_state.target_x - odom_state.global_x;
    float world_y_diff = pursuit_state.target_y - odom_state.global_y;
    float relative_y = (world_y_diff * pb_fast_cos(odom_state.global_h)) - (world_x_diff * pb_fast_sin(odom_state.global_h));
    float dist_sq = (world_x_diff * world_x_diff) + (world_y_diff * world_y_diff);

    if (relative_y > 0.001f || relative_y < -0.001f) {
        return dist_sq / (2.0f * relative_y);
    }
    return 0.0f;
}

static float evaluate_path_curvature(void) {
    float dx = evaluate_x(pursuit_state.t_lookahead, 1);
    float dy = evaluate_y(pursuit_state.t_lookahead, 1);
    float ddx = evaluate_x(pursuit_state.t_lookahead, 2);
    float ddy = evaluate_y(pursuit_state.t_lookahead, 2);

    float curvature = 0.0f;
    float den_b = (dx * dx) + (dy * dy);
    if (den_b > 1e-12f) {
        float num = fabsf(dx * ddy - dy * ddx);
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

static void execute_speed_control(float turning_radius, float path_curvature, float time_passed, float speed_cap) {

    float curvature_span = pursuit_state.min_curvature - pursuit_state.max_curvature;
    float local_max_speed;
    if (curvature_span > -1e-9f && curvature_span < 1e-9f) {
        local_max_speed = pursuit_state.max_speed;
    } else {
        local_max_speed = pursuit_state.min_speed +
            (path_curvature - pursuit_state.max_curvature) *
            (pursuit_state.max_speed - pursuit_state.min_speed) / curvature_span;
    }
    if (local_max_speed > pursuit_state.max_speed) {
        local_max_speed = pursuit_state.max_speed;
    }
    if (local_max_speed < pursuit_state.min_speed) {
        local_max_speed = pursuit_state.min_speed;
    }
    if (local_max_speed > speed_cap) {
        local_max_speed = speed_cap;
    }

    float local_base_speed = local_max_speed * pursuit_state.base_speed_percentage;
    float right_target, left_target;

    if (turning_radius == 0.0f || odom_state.inv_track <= 0.0f) {
        // fixed robot stopping when reaching the point
        right_target = local_base_speed;
        left_target = local_base_speed;
    } else {
        float track_half = (1.0f / odom_state.inv_track) * 0.5f;
        right_target = local_base_speed * (turning_radius + track_half) / turning_radius;
        left_target = local_base_speed * (turning_radius - track_half) / turning_radius;

        float peak = fabsf(right_target) > fabsf(left_target) ? fabsf(right_target) : fabsf(left_target);
        if (peak > pursuit_state.max_speed && peak > 0.0f) {
            float scale = pursuit_state.max_speed / peak;
            right_target *= scale;
            left_target *= scale;
        }
    }

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

    // fixed scaling
    float current_robot_speed = (pursuit_state.right_motor_speed + pursuit_state.left_motor_speed) * 0.5f;
    if (current_robot_speed > pursuit_state.max_speed) {
        current_robot_speed = pursuit_state.max_speed;
    }
    if (current_robot_speed < pursuit_state.min_speed) {
        current_robot_speed = pursuit_state.min_speed;
    }

    float speed_span = pursuit_state.max_speed - pursuit_state.min_speed;
    if (speed_span > 1e-6f) {
        pursuit_state.lookahead = pursuit_state.min_lookahead +
            (current_robot_speed - pursuit_state.min_speed) *
            (pursuit_state.max_lookahead - pursuit_state.min_lookahead) / speed_span;
    } else {
        pursuit_state.lookahead = pursuit_state.min_lookahead;
    }

    int32_t cmd_left = (int32_t)pursuit_state.left_motor_speed;
    int32_t cmd_right = (int32_t)pursuit_state.right_motor_speed;
    if (!cmd_valid || cmd_left != last_cmd_left) {
        pbio_servo_run_forever(odom_state.left_servo, cmd_left);
        last_cmd_left = cmd_left;
    }
    if (!cmd_valid || cmd_right != last_cmd_right) {
        pbio_servo_run_forever(odom_state.right_servo, cmd_right);
        last_cmd_right = cmd_right;
    }
    cmd_valid = true;
}

void pb_pursuit_step(float time_passed) {
    if (!pursuit_state.running || !odom_state.left_servo || !odom_state.right_servo) {
        return;
    }

    target_point_approximation();

    if (pursuit_state.t_lookahead >= (float)pursuit_state.total_splines) {
        finishing = true;
    }

    float speed_cap = pursuit_state.max_speed;
    if (finishing) {
        float ex = end_x - odom_state.global_x;
        float ey = end_y - odom_state.global_y;
        float dist = sqrtf(ex * ex + ey * ey);
        float forward = ex * pb_fast_cos(odom_state.global_h) + ey * pb_fast_sin(odom_state.global_h);

        if (dist < PB_PURSUIT_STOP_TOLERANCE_MM || forward < 0.0f) {
            experimental_stop_pursuit();
            return;
        }

        pursuit_state.target_x = end_x;
        pursuit_state.target_y = end_y;

        if (odom_state.deg_to_mm > 0.0f) {
            speed_cap = sqrtf(2.0f * PB_PURSUIT_DECEL_MARGIN *
                pursuit_state.max_per_motor_acceleration * dist / odom_state.deg_to_mm);
        }
    }

    float turning_radius = calculate_pure_pursuit();
    float path_curvature = evaluate_path_curvature();
    execute_speed_control(turning_radius, path_curvature, time_passed, speed_cap);
}

void pb_pursuit_reset(void) {
    memset(&pursuit_state, 0, sizeof(pursuit_state));
    finishing = false;
    cmd_valid = false;
}

mp_obj_t experimental_start_pursuit(size_t n_args, const mp_obj_t *args) {

    // cant run pursuit without odom
    if (!odom_state.running) {
        mp_raise_ValueError(MP_ERROR_TEXT("call start_odometry() first"));
    }

    if (pursuit_state.running) {
        experimental_stop_pursuit();
    }

    size_t num_splines;
    mp_obj_t *splines_arr;
    mp_obj_get_array(args[0], &num_splines, &splines_arr);
    if (num_splines == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("no splines given"));
    }
    if (num_splines > PB_PURSUIT_MAX_SPLINES) {
        mp_raise_ValueError(MP_ERROR_TEXT("too many splines"));
    }
    pursuit_state.total_splines = (uint16_t)num_splines;

    for (size_t i = 0; i < num_splines; i++)
    {
        size_t num_coeffs;
        mp_obj_t *coeffs_arr;
        mp_obj_get_array(splines_arr[i], &num_coeffs, &coeffs_arr);
        if (num_coeffs < 8) {
            mp_raise_ValueError(MP_ERROR_TEXT("each spline needs 8 coefficients"));
        }
        for (size_t j = 0; j < 8; j++)
        {
            pursuit_state.spline_coefficients[i][j] = mp_obj_get_float(coeffs_arr[j]);
        }
    }

    size_t db_len;
    mp_obj_t *db_arr;
    mp_obj_get_array(args[1], &db_len, &db_arr);
    if (db_len < 6) {
        mp_raise_ValueError(MP_ERROR_TEXT("drive_base needs 6 entries"));
    }
    pursuit_state.max_speed = mp_obj_get_float(db_arr[2]);
    pursuit_state.min_speed = mp_obj_get_float(db_arr[3]);
    pursuit_state.base_speed_percentage = mp_obj_get_float(db_arr[4]);
    pursuit_state.max_per_motor_acceleration = mp_obj_get_float(db_arr[5]);

    if (pursuit_state.max_speed < pursuit_state.min_speed) {
        mp_raise_ValueError(MP_ERROR_TEXT("max_speed below min_speed"));
    }
    if (pursuit_state.max_per_motor_acceleration <= 0.0f) {
        mp_raise_ValueError(MP_ERROR_TEXT("acceleration must be positive"));
    }

    size_t tun_len;
    mp_obj_t *tun_arr;
    mp_obj_get_array(args[2], &tun_len, &tun_arr);
    if (tun_len < 5) {
        mp_raise_ValueError(MP_ERROR_TEXT("tuning needs 5 entries"));
    }

    float radius_a = mp_obj_get_float(tun_arr[0]);
    float radius_b = mp_obj_get_float(tun_arr[1]);
    if (radius_a <= 0.0f || radius_b <= 0.0f) {
        mp_raise_ValueError(MP_ERROR_TEXT("tuning radii must be positive"));
    }
    pursuit_state.min_curvature = 1.0f / radius_a;
    pursuit_state.max_curvature = 1.0f / radius_b;
    if (pursuit_state.min_curvature > pursuit_state.max_curvature) {
        float swap = pursuit_state.min_curvature;
        pursuit_state.min_curvature = pursuit_state.max_curvature;
        pursuit_state.max_curvature = swap;
    }

    pursuit_state.max_lookahead = mp_obj_get_float(tun_arr[2]);
    pursuit_state.min_lookahead = mp_obj_get_float(tun_arr[3]);

    int iterations = mp_obj_get_int(tun_arr[4]);
    if (iterations < 1) {
        iterations = 1;
    }
    if (iterations > 255) {
        iterations = 255;
    }
    pursuit_state.total_newton_iterations = (uint8_t)iterations;

    finishing = false;
    cmd_valid = false;
    end_x = evaluate_x((float)pursuit_state.total_splines, 0);
    end_y = evaluate_y((float)pursuit_state.total_splines, 0);

    pursuit_state.t_lookahead = 0.0f;
    pursuit_state.lookahead = pursuit_state.min_lookahead;
    pursuit_state.left_motor_speed = 0.0f;
    pursuit_state.right_motor_speed = 0.0f;
    pursuit_state.target_x = odom_state.global_x;
    pursuit_state.target_y = odom_state.global_y;
    pursuit_state.running = true;
    return mp_const_none;
}

mp_obj_t experimental_stop_pursuit(void) {
    pursuit_state.running = false;
    finishing = false;
    cmd_valid = false;
    pursuit_state.left_motor_speed = 0.0f;
    pursuit_state.right_motor_speed = 0.0f;
    if (odom_state.left_servo && odom_state.right_servo) {
        pbio_servo_stop(odom_state.left_servo, PBIO_CONTROL_ON_COMPLETION_BRAKE);
        pbio_servo_stop(odom_state.right_servo, PBIO_CONTROL_ON_COMPLETION_BRAKE);
    }
    return mp_const_none;
}

#endif // PYBRICKS_PY_EXPERIMENTAL
