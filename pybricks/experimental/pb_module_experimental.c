// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2021 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"

#include <math.h>

// Math Constants - Hardcoded for speed
static const float PI_F         = 3.1415926535f;
static const float TWO_PI_F     = 6.2831853071f;
static const float HALF_PI_F    = 1.5707963267f;
static const float INV_TWO_PI_F = 0.1591549431f;

// -----------------------------------------------------------------------------
// Min-Maxed Internal Engines
// -----------------------------------------------------------------------------

// Core Polynomial - Inlined to remove call overhead
static inline float fast_sin_poly(float x) {
    float x2 = x * x;
    // Horner's Method: reduces the number of multiplications
    return x * (1.0f + x2 * (-0.1666665f + x2 * 0.0083322f));
}

static inline float fast_sin_internal(float theta) {
    // Fast range reduction
    float quot = theta * INV_TWO_PI_F;
    float x = theta - (float)((int)(quot + (quot > 0 ? 0.5f : -0.5f))) * TWO_PI_F;

    // Symmetry reduction
    if (x > HALF_PI_F) { x = PI_F - x; }
    else if (x < -HALF_PI_F) { x = -PI_F - x; }

    return fast_sin_poly(x);
}

static inline float fast_atan2_internal(float y, float x) {
    float ay = fabsf(y) + 1e-10f; 
    float ax = fabsf(x);
    float z, angle;

    if (ax >= ay) {
        z = y / ax;
        angle = (0.7853982f + 0.273f * (1.0f - fabsf(z))) * z;
    } else {
        z = x / ay;
        angle = 1.5707963f - (0.7853982f + 0.273f * (1.0f - fabsf(z))) * z;
    }

    if (x < 0.0f) {
        angle += (y >= 0.0f) ? PI_F : -PI_F;
    }
    return angle;
}

// -----------------------------------------------------------------------------
// MicroPython Wrappers
// -----------------------------------------------------------------------------

static mp_obj_t experimental_sin(mp_obj_t theta_in) {
    return mp_obj_new_float_from_f(fast_sin_internal(mp_obj_get_float(theta_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_sin_obj, experimental_sin);

static mp_obj_t experimental_cos(mp_obj_t theta_in) {
    return mp_obj_new_float_from_f(fast_sin_internal(mp_obj_get_float(theta_in) + HALF_PI_F));
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_cos_obj, experimental_cos);

// MIN-MAX: Using FUN_OBJ_2 instead of FUN_OBJ_KW to eliminate keyword parsing overhead
static mp_obj_t experimental_atan2(mp_obj_t y_in, mp_obj_t x_in) {
    return mp_obj_new_float_from_f(fast_atan2_internal(mp_obj_get_float(y_in), mp_obj_get_float(x_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(experimental_atan2_obj, experimental_atan2);

// -----------------------------------------------------------------------------
// Benchmark
// -----------------------------------------------------------------------------

static mp_obj_t experimental_benchmark_internal(mp_obj_t n_in) {
    int32_t n = mp_obj_get_int(n_in);
    volatile float result = 0.0f; 
    uint32_t start = mp_hal_ticks_ms();
    
    for (int32_t i = 0; i < n; i++) {
        result += fast_sin_internal(1.23f);
        result += fast_sin_internal(1.23f + HALF_PI_F);
        result += fast_atan2_internal(1.23f, 1.23f);
    }
    
    uint32_t end = mp_hal_ticks_ms();
    uint32_t total_ms = end - start;
    float ns_per_op = (n > 0) ? ((float)total_ms * 1000000.0f) / n : 0;

    mp_obj_t tuple[2] = { mp_obj_new_int(total_ms), mp_obj_new_float(ns_per_op) };
    return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_benchmark_internal_obj, experimental_benchmark_internal);

// -----------------------------------------------------------------------------
// Module Registry
// -----------------------------------------------------------------------------

static const mp_rom_map_elem_t experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),           MP_ROM_QSTR(MP_QSTR_experimental) },
    { MP_ROM_QSTR(MP_QSTR_sin),                MP_ROM_PTR(&experimental_sin_obj) },
    { MP_ROM_QSTR(MP_QSTR_cos),                MP_ROM_PTR(&experimental_cos_obj) },
    { MP_ROM_QSTR(MP_QSTR_atan2),              MP_ROM_PTR(&experimental_atan2_obj) },
    { MP_ROM_QSTR(MP_QSTR_benchmark_internal), MP_ROM_PTR(&experimental_benchmark_internal_obj) },
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