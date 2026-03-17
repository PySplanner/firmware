// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2026 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>

#include <pbio/tacho.h>
#include <pbio/drivebase.h>

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    #define IS_CORTEX_M 1
    #define ACCEL_RAM __attribute__((section(".data"), noinline))
#else
    #define IS_CORTEX_M 0
    #define ACCEL_RAM
#endif

// Manually defining internal structures to ensure compilation across builds
typedef struct _pb_type_Motor_obj_t {
    mp_obj_base_t base;
    pbio_tacho_t *tacho;
} pb_type_Motor_obj_t;

typedef struct _pb_type_DriveBase_obj_t {
    mp_obj_base_t base;
    pbio_drivebase_t *db;
} pb_type_DriveBase_obj_t;

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

// -----------------------------------------------------------------------------
// High-Speed Direct-Tacho Odometry
// -----------------------------------------------------------------------------
static mp_obj_t experimental_odometry_benchmark(size_t n_args, const mp_obj_t *args) {
    int num_iters = mp_obj_get_int(args[0]);
    float wheel_circ = mp_obj_get_float(args[1]);

    pb_type_Motor_obj_t *right_motor = (pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(args[2]);
    pb_type_Motor_obj_t *left_motor = (pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(args[3]);
    pb_type_DriveBase_obj_t *db_obj = (pb_type_DriveBase_obj_t *)MP_OBJ_TO_PTR(args[4]);

    // mm per degree
    float deg_to_mm = wheel_circ / 360.0f;
    float robot_x = 0.0f, robot_y = 0.0f;
    
    pbio_angle_t ang_l, ang_r;
    int32_t h_mdeg;

    // Initial state capture
    pbio_tacho_get_angle(left_motor->tacho, &ang_l);
    pbio_tacho_get_angle(right_motor->tacho, &ang_r);
    pbio_drivebase_get_state_user(db_obj->db, NULL, NULL, &h_mdeg, NULL);

    float last_l_mm = ((float)ang_l.rotations * 360.0f + (float)ang_l.millidegrees / 1000.0f) * deg_to_mm;
    float last_r_mm = ((float)ang_r.rotations * 360.0f + (float)ang_r.millidegrees / 1000.0f) * deg_to_mm;
    float last_lin = (last_l_mm + last_r_mm) / 2.0f;
    float last_heading = (float)h_mdeg / 1000.0f;

    uint32_t start_time = mp_hal_ticks_ms();

    for (int i = 0; i < num_iters; i++) {
        // DIRECT SENSOR READS
        pbio_tacho_get_angle(left_motor->tacho, &ang_l);
        pbio_tacho_get_angle(right_motor->tacho, &ang_r);
        pbio_drivebase_get_state_user(db_obj->db, NULL, NULL, &h_mdeg, NULL);

        // Convert to mm
        float cur_l_mm = ((float)ang_l.rotations * 360.0f + (float)ang_l.millidegrees / 1000.0f) * deg_to_mm;
        float cur_r_mm = ((float)ang_r.rotations * 360.0f + (float)ang_r.millidegrees / 1000.0f) * deg_to_mm;
        
        float cur_lin = (cur_l_mm + cur_r_mm) / 2.0f;
        float cur_heading = (float)h_mdeg / 1000.0f;

        float linear_delta = cur_lin - last_lin;
        float heading_delta = cur_heading - last_heading;

        // Only update if we actually moved
        if (fabsf(linear_delta) > 0.0001f) {
            // Average heading during the delta step for arc integration
            float avg_h_rad = (last_heading + (heading_delta / 2.0f)) * 0.0174532925f;
            
            robot_x += linear_delta * fast_sin_internal(avg_h_rad + HALF_PI_F);
            robot_y += linear_delta * fast_sin_internal(avg_h_rad);
        }

        last_lin = cur_lin;
        last_heading = cur_heading;

        // Stability yield
        if ((i % 2000) == 0) {
            mp_handle_pending(true);
            mp_hal_delay_ms(1);
        }
    }

    uint32_t dur = mp_hal_ticks_ms() - start_time;
    float duration = (float)dur / 1000.0f;

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