// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2026 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>

// Direct access to Pybricks internal types
#include "pybricks/pupdevices/pb_type_pupdevices_motor.h"
#include "pybricks/robotics/pb_type_robotics_drivebase.h"
#include <pbio/tacho.h>

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    #define IS_CORTEX_M 1
    #define ACCEL_RAM __attribute__((section(".data"), noinline))
#else
    #define IS_CORTEX_M 0
    #define ACCEL_RAM
#endif

// Constants
static const float PI_F = 3.141592653589793f;
static const float TWO_PI_F = 6.283185307179586f;
static const float HALF_PI_F = 1.570796326794896f;
static const float INV_TWO_PI_F = 0.159154943091895f;

// -----------------------------------------------------------------------------
// Core Math Engine (Lasse Schlör Absolute Error Optimized)
// -----------------------------------------------------------------------------
ACCEL_RAM static float fast_sin_internal(float theta) {
    float x = theta * INV_TWO_PI_F;
    x = theta - (float)((int)(x + (x > 0 ? 0.5f : -0.5f))) * TWO_PI_F;

    if (x > HALF_PI_F) {
        x = PI_F - x;
    } else if (x < -HALF_PI_F) {
        x = -PI_F - x;
    }

    float x2 = x * x;
    #if IS_CORTEX_M
    float res = -0.0001848814f;
    res = 0.0083119000f + (x2 * res);
    res = -0.1666555409f + (x2 * res);
    return x * (0.9999990609f + (x2 * res));
    #else
    return x * (0.9999990609f + x2 * (-0.1666555409f + x2 * (0.0083119000f + x2 * -0.0001848814f)));
    #endif
}

// Helper to safely unpack hardware values
static float get_float_from_obj(mp_obj_t obj) {
    if (MP_OBJ_IS_SMALL_INT(obj)) {
        return (float)MP_OBJ_SMALL_INT_VALUE(obj);
    } else if (mp_obj_is_type(obj, &mp_type_float)) {
        return mp_obj_get_float(obj);
    } else {
        return (float)mp_obj_get_int(obj);
    }
}

// -----------------------------------------------------------------------------
// The "Bare Metal" Odometry Benchmark
// -----------------------------------------------------------------------------
static mp_obj_t experimental_odometry_benchmark(size_t n_args, const mp_obj_t *args) {
    // 1. Unpack Arguments as RAW C-STRUCTS
    int num_iters = mp_obj_get_int(args[0]);
    float wheel_circ = get_float_from_obj(args[1]);

    // Cast Python objects directly to their internal C types
    pb_type_Motor_obj_t *right_motor = (pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(args[2]);
    pb_type_Motor_obj_t *left_motor = (pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(args[3]);
    pb_type_DriveBase_obj_t *db = (pb_type_DriveBase_obj_t *)MP_OBJ_TO_PTR(args[4]);

    float deg_to_mm = wheel_circ / 720.0f;
    float robot_x = 0.0f, robot_y = 0.0f;
    float last_linear = 0.0f, last_heading = 0.0f;

    uint32_t start_time = mp_hal_ticks_ms();

    // 2. The High-Speed Loop (Zero Python Callbacks)
    for (int i = 0; i < num_iters; i++) {
        // Direct pointer access to encoder angles
        int32_t r_ang, l_ang;
        pbio_tacho_get_angle(right_motor->tacho, &r_ang);
        pbio_tacho_get_angle(left_motor->tacho, &l_ang);

        // Direct pointer access to DriveBase heading (no .angle() call)
        // Note: heading is already a float in the DriveBase struct
        float robot_heading = db->heading;

        float current_linear = (float)(r_ang + l_ang) * deg_to_mm;
        float linear = current_linear - last_linear;
        float heading_difference = robot_heading - last_heading;

        last_linear = current_linear;
        last_heading = robot_heading;

        if (fabsf(linear) > 0.0f) {
            float avg_heading_deg = last_heading - (heading_difference / 2.0f);
            float avg_heading_rad = avg_heading_deg * 0.0174532925f;

            robot_x += linear * fast_sin_internal(avg_heading_rad + HALF_PI_F);
            robot_y += linear * fast_sin_internal(avg_heading_rad);
        }

        if ((i % 1000) == 0) {
            mp_handle_pending(true);
        }
    }

    uint32_t duration_ms = mp_hal_ticks_ms() - start_time;
    float duration = (float)duration_ms / 1000.0f;

    mp_obj_t tuple[5] = {
        mp_obj_new_float_from_f(duration),
        mp_obj_new_int(num_iters),
        mp_obj_new_float_from_f((float)num_iters / duration),
        mp_obj_new_float_from_f(robot_x),
        mp_obj_new_float_from_f(robot_y)
    };
    return mp_obj_new_tuple(5, tuple);
}

static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(experimental_odometry_benchmark_obj, 5, 5, experimental_odometry_benchmark);

static const mp_rom_map_elem_t experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_experimental) },
    { MP_ROM_QSTR(MP_QSTR_odometry_benchmark), MP_ROM_PTR(&experimental_odometry_benchmark_obj) },
};
static MP_DEFINE_CONST_DICT(pb_module_experimental_globals, experimental_globals_table);

const mp_obj_module_t pb_module_experimental = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pb_module_experimental_globals,
};

#if !MICROPY_MODULE_BUILTIN_SUBPACKAGES
MP_REGISTER_MODULE(MP_QSTR_pybricks_dot_experimental, pb_module_experimental);
#endif

#endif // PYBRICKS_PY_EXPERIMENTAL