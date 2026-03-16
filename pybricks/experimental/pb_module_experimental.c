// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2026 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>

// Direct Hardware Access Headers
#include <pbio/tacho.h>
#include <pbio/port.h>

// Architecture Detection & Optimization Macros
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    #define IS_CORTEX_M 1
    #define ACCEL_RAM __attribute__((section(".data"), noinline))
    #define DWT_CONTROL  (*((volatile uint32_t*)0xE0001000))
    #define DWT_CYCCNT   (*((volatile uint32_t*)0xE0001004))
    #define DEMCR         (*((volatile uint32_t*)0xE000EDFC))
#else
    #define IS_CORTEX_M 0
    #define ACCEL_RAM   // EV3/Linux handles RAM loading automatically
#endif

// Constants
static const float PI_F          = 3.141592653589793f;
static const float TWO_PI_F      = 6.283185307179586f;
static const float HALF_PI_F     = 1.570796326794896f;
static const float INV_TWO_PI_F  = 0.159154943091895f;

// -----------------------------------------------------------------------------
// Core Math Engines (Lasse Schlör Absolute Error MiniMax Coefficients)
// -----------------------------------------------------------------------------

ACCEL_RAM static float fast_sin_internal(float theta) {
    // 1. Precise Range Reduction to [-PI, PI]
    float x = theta * INV_TWO_PI_F;
    x = theta - (float)((int)(x + (x > 0 ? 0.5f : -0.5f))) * TWO_PI_F;

    // 2. Quadrant folding to [-PI/2, PI/2]
    if (x > HALF_PI_F) { x = PI_F - x; }
    else if (x < -HALF_PI_F) { x = -PI_F - x; }

    float x2 = x * x;
    float res;

    #if IS_CORTEX_M
        // Spike Prime: Horner's Scheme for VMLA.F32 pipeline
        res = -0.0001848814f; 
        res = 0.0083119000f + (x2 * res);
        res = -0.1666555409f + (x2 * res);
        res = 0.9999990609f + (x2 * res);
    #else
        // EV3: Minimized Absolute Error (~9.33e-7)
        res = 0.9999990609f + x2 * (-0.1666555409f + x2 * (0.0083119000f + x2 * -0.0001848814f));
    #endif
    
    return x * res;
}

ACCEL_RAM static float fast_atan2_internal(float y, float x) {
    if (x == 0.0f && y == 0.0f) return 0.0f;
    float abs_y = fabsf(y) + 1e-10f;
    float abs_x = fabsf(x);
    float angle;
    float r = (abs_x >= abs_y) ? (y / x) : (x / y);
    float den = 1.0f + (r * r * 0.28086f);
    float res_atan = r * (1.0f / den);

    if (abs_x >= abs_y) {
        angle = res_atan;
        if (x < 0.0f) angle += (y >= 0.0f) ? PI_F : -PI_F;
    } else {
        angle = (y > 0.0f ? HALF_PI_F : -HALF_PI_F) - res_atan;
    }
    return angle;
}

// -----------------------------------------------------------------------------
// Python Wrappers & Hardware Benchmarks
// -----------------------------------------------------------------------------

static mp_obj_t experimental_sin(mp_obj_t theta_in) {
    return mp_obj_new_float_from_f(fast_sin_internal(mp_obj_get_float(theta_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_sin_obj, experimental_sin);

static mp_obj_t experimental_cos(mp_obj_t theta_in) {
    return mp_obj_new_float_from_f(fast_sin_internal(mp_obj_get_float(theta_in) + HALF_PI_F));
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_cos_obj, experimental_cos);

static mp_obj_t experimental_atan2(mp_obj_t y_in, mp_obj_t x_in) {
    return mp_obj_new_float_from_f(fast_atan2_internal(mp_obj_get_float(y_in), mp_obj_get_float(x_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(experimental_atan2_obj, experimental_atan2);

// The NEW Hardware-Polling Odometry Benchmark
static mp_obj_t experimental_odometry_benchmark(mp_obj_t num_iters_in, mp_obj_t wheel_circ_in) {
    int num_iters = mp_obj_get_int(num_iters_in);
    float wheel_circ = mp_obj_get_float(wheel_circ_in);
    float deg_to_mm = wheel_circ / 360.0f;

    float robot_x = 0.0f, robot_y = 0.0f;
    int32_t last_left = 0, last_right = 0;
    float heading = 0.0f;
    
    // Initial fetch (Assuming Left=A, Right=D as per your robot)
    pbio_tacho_get_count(PBIO_PORT_ID_A, &last_left);
    pbio_tacho_get_count(PBIO_PORT_ID_D, &last_right);

    uint32_t start_time = mp_hal_ticks_ms();

    for (int i = 0; i < num_iters; i++) {
        int32_t cur_left, cur_right;
        pbio_tacho_get_count(PBIO_PORT_ID_A, &cur_left);
        pbio_tacho_get_count(PBIO_PORT_ID_D, &cur_right);

        float d_left = (float)(cur_left - last_left) * deg_to_mm;
        float d_right = (float)(cur_right - last_right) * deg_to_mm;
        float linear_delta = (d_left + d_right) / 2.0f;
        float heading_delta = (d_right - d_left) / 96.0f; // 96mm axle track

        if (fabsf(linear_delta) > 0.0f) {
            float avg_h = heading + (heading_delta / 2.0f);
            robot_x += linear_delta * fast_sin_internal(avg_h + HALF_PI_F); // Cos
            robot_y += linear_delta * fast_sin_internal(avg_h); // Sin
        }

        heading += heading_delta;
        last_left = cur_left;
        last_right = cur_right;

        if ((i % 1000) == 0) { MICROPY_EVENT_POLL_HOOK }
    }

    uint32_t duration_ms = mp_hal_ticks_ms() - start_time;
    float duration = duration_ms / 1000.0f;

    mp_obj_t tuple[5] = {
        mp_obj_new_float_from_f(duration),
        mp_obj_new_int(num_iters),
        mp_obj_new_float_from_f((float)num_iters / duration),
        mp_obj_new_float_from_f(robot_x),
        mp_obj_new_float_from_f(robot_y)
    };
    return mp_obj_new_tuple(5, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_2(experimental_odometry_benchmark_obj, experimental_odometry_benchmark);

// Original CPU-only benchmark for pure math testing
static mp_obj_t experimental_benchmark_math(mp_obj_t seed_in) {
    float seed = mp_obj_get_float(seed_in);
    #if IS_CORTEX_M
        DEMCR |= 0x01000000; DWT_CONTROL |= 1;
        DWT_CYCCNT = 0;
        uint32_t start = DWT_CYCCNT;
        volatile float res = fast_sin_internal(seed);
        res = fast_sin_internal(res + 0.01f); 
        __asm volatile ("dsb"); 
        return mp_obj_new_int((DWT_CYCCNT - start) / 2);
    #else
        uint32_t t0 = mp_hal_ticks_ms();
        volatile float res = seed;
        for(int i=0; i<50000; i++) { res = fast_sin_internal(res + 0.0001f); }
        return mp_obj_new_int(((mp_hal_ticks_ms() - t0) * 1000000) / 50000);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_benchmark_math_obj, experimental_benchmark_math);

// Module Globals Table
static const mp_rom_map_elem_t experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),             MP_ROM_QSTR(MP_QSTR_experimental) },
    { MP_ROM_QSTR(MP_QSTR_sin),                  MP_ROM_PTR(&experimental_sin_obj) },
    { MP_ROM_QSTR(MP_QSTR_cos),                  MP_ROM_PTR(&experimental_cos_obj) },
    { MP_ROM_QSTR(MP_QSTR_atan2),                MP_ROM_PTR(&experimental_atan2_obj) },
    { MP_ROM_QSTR(MP_QSTR_benchmark_math),       MP_ROM_PTR(&experimental_benchmark_math_obj) },
    { MP_ROM_QSTR(MP_QSTR_odometry_benchmark),   MP_ROM_PTR(&experimental_odometry_benchmark_obj) },
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