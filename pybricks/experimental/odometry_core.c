#include "py/mpconfig.h"
#include "py/mphal.h"
#include "py/runtime.h"

// This must be here for the compiler to see pb_fast_sin/cos
#include "pybricks/experimental/platform_math.h"
#include "pybricks/experimental/odometry.h"

mp_obj_t calculate_odometry(int num_iters, float wheel_circ, float axle_track, mp_obj_t right_angle_func, mp_obj_t left_angle_func) {
    float deg_to_mm = wheel_circ * 0.0027777778f;
    float inv_axle_track = 1.0f / axle_track;
    float rx = 0.0f, ry = 0.0f, rh = 0.0f;

    int32_t last_r = mp_obj_get_int(mp_call_function_0(right_angle_func));
    int32_t last_l = mp_obj_get_int(mp_call_function_0(left_angle_func));
    uint32_t start_time = mp_hal_ticks_ms();

    for (int i = 0; i < num_iters; i++) {
        int32_t cur_r = mp_obj_get_int(mp_call_function_0(right_angle_func));
        int32_t cur_l = mp_obj_get_int(mp_call_function_0(left_angle_func));

        float dR = (float)(cur_r - last_r) * deg_to_mm;
        float dL = (float)(cur_l - last_l) * deg_to_mm;
        float dD = (dR + dL) * 0.5f;
        float dH = (dR - dL) * inv_axle_track;

        if (dD != 0.0f || dH != 0.0f) {
            float avg_h = rh + (dH * 0.5f);
            // These now use the optimized Spike polynomials
            rx += dD * pb_fast_cos(avg_h);
            ry += dD * pb_fast_sin(avg_h);
            rh += dH;
        }
        last_r = cur_r; last_l = cur_l;

        if ((i & 0x3FF) == 0) {
            mp_handle_pending(true);
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