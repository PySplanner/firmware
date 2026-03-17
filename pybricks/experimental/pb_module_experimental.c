// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2026 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>

// Pybricks Hardware I/O Headers
#include <pbio/tacho.h>
#include <pbio/drivebase.h>
#include <pbio/port.h>

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    #define IS_CORTEX_M 1
    #define ACCEL_RAM __attribute__((section(".data"), noinline))
#else
    #define IS_CORTEX_M 0
    #define ACCEL_RAM
#endif

// -----------------------------------------------------------------------------
// Core Math Engine (Optimized MiniMax Polynomial)
// -----------------------------------------------------------------------------
ACCEL_RAM static float fast_sin_internal(float theta) {
    float x = theta * 0.159154943f; // INV_TWO_PI
    x = theta - (float)((int)(x + (x > 0 ? 0.5f : -0.5f))) * 6.2831853f; // TWO_PI
    if (x > 1.5707963f) x = 3.1415926f - x; // PI - x
    else if (x < -1.5707963f) x = -3.1415926f - x; // -PI - x
    float x2 = x * x;
    return x * (0.99999906f + x2 * (-0.16665554f + x2 * (0.00831190f + x2 * -0.00018488f)));
}

// -----------------------------------------------------------------------------
// High-Speed Hardware-Layer Odometry
// -----------------------------------------------------------------------------
static mp_obj_t experimental_odometry_benchmark(size_t n_args, const mp_obj_t *args) {
    int num_iters = mp_obj_get_int(args[0]);
    float wheel_circ = mp_obj_get_float(args[1]);

    // 1. Unpack raw Port IDs from Python (e.g., ord('A') -> 65)
    pbio_port_id_t port_right = (pbio_port_id_t)mp_obj_get_int(args[2]);
    pbio_port_id_t port_left  = (pbio_port_id_t)mp_obj_get_int(args[3]);

    // 2. Bind directly to the hardware tacho drivers
    pbio_tacho_t *tacho_r;
    pbio_tacho_t *tacho_l;
    
    // PBIO returns 0 (PBIO_SUCCESS) if the hardware exists on that port
    if (pbio_tacho_get_tacho(port_right, &tacho_r) != 0 || 
        pbio_tacho_get_tacho(port_left, &tacho_l) != 0) {
        mp_printf(&mp_plat_print, "C-ERROR: Failed to bind to hardware tachos on specified ports.\n");
        mp_obj_t err[5] = {mp_obj_new_float_from_f(0), mp_obj_new_int(0), mp_obj_new_float_from_f(0), mp_obj_new_float_from_f(0), mp_obj_new_float_from_f(0)};
        return mp_obj_new_tuple(5, err);
    }

    // 3. Unpack the DriveBase heading function
    mp_obj_t db_heading_func = args[4];

    float deg_to_mm = wheel_circ / 360.0f;
    float robot_x = 0.0f, robot_y = 0.0f;
    
    pbio_angle_t ang_l, ang_r;
    
    // Capture initial hardware state
    pbio_tacho_get_angle(tacho_l, &ang_l);
    pbio_tacho_get_angle(tacho_r, &ang_r);
    int32_t last_heading_deg = mp_obj_get_int(mp_call_function_0(db_heading_func));

    float last_l_mm = ((float)ang_l.rotations * 360.0f + (float)ang_l.millidegrees / 1000.0f) * deg_to_mm;
    float last_r_mm = ((float)ang_r.rotations * 360.0f + (float)ang_r.millidegrees / 1000.0f) * deg_to_mm;
    float last_lin = (last_l_mm + last_r_mm) / 2.0f;

    uint32_t start_time = mp_hal_ticks_ms();

    for (int i = 0; i < num_iters; i++) {
        // Direct C-speed hardware reads
        pbio_tacho_get_angle(tacho_l, &ang_l);
        pbio_tacho_get_angle(tacho_r, &ang_r);
        
        // Single VM call for complex gyro fusion heading
        int32_t cur_heading_deg = mp_obj_get_int(mp_call_function_0(db_heading_func));

        float cur_l_mm = ((float)ang_l.rotations * 360.0f + (float)ang_l.millidegrees / 1000.0f) * deg_to_mm;
        float cur_r_mm = ((float)ang_r.rotations * 360.0f + (float)ang_r.millidegrees / 1000.0f) * deg_to_mm;
        float cur_lin = (cur_l_mm + cur_r_mm) / 2.0f;

        float linear_delta = cur_lin - last_lin;
        float heading_delta = (float)(cur_heading_deg - last_heading_deg);

        if (fabsf(linear_delta) > 0.0001f) {
            float avg_h_rad = ((float)last_heading_deg + (heading_delta / 2.0f)) * 0.01745329f;
            robot_x += linear_delta * fast_sin_internal(avg_h_rad + 1.5707963f); // cos
            robot_y += linear_delta * fast_sin_internal(avg_h_rad); // sin
        }

        last_lin = cur_lin;
        last_heading_deg = cur_heading_deg;

        // Yield to maintain BLE connection and reset watchdog
        if ((i % 2000) == 0) {
            mp_handle_pending(true);
            mp_hal_delay_ms(1);
        }
    }

    uint32_t dur = mp_hal_ticks_ms() - start_time;
    float duration_s = (float)dur / 1000.0f;
    
    mp_obj_t tuple[5] = {
        mp_obj_new_float_from_f(duration_s),
        mp_obj_new_int(num_iters),
        mp_obj_new_float_from_f((float)num_iters / duration_s),
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