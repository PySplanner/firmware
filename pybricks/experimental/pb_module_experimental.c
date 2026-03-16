// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2021 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>

static const float PI_F         = 3.1415926535f;
static const float TWO_PI_F     = 6.2831853071f;
static const float HALF_PI_F    = 1.5707963267f;
static const float INV_TWO_PI_F = 0.1591549431f;

// -----------------------------------------------------------------------------
// Internal Engines (Inlined)
// -----------------------------------------------------------------------------

static inline float fast_sin_poly(float x) {
    float x2 = x * x;
    return x * (1.0f + x2 * (-0.1666665f + x2 * 0.0083322f));
}

static inline float fast_sin_internal(float theta) {
    float quot = theta * INV_TWO_PI_F;
    float x = theta - (float)((int)(quot + (quot > 0 ? 0.5f : -0.5f))) * TWO_PI_F;
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

static mp_obj_t experimental_atan2(mp_obj_t y_in, mp_obj_t x_in) {
    return mp_obj_new_float_from_f(fast_atan2_internal(mp_obj_get_float(y_in), mp_obj_get_float(x_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(experimental_atan2_obj, experimental_atan2);

// -----------------------------------------------------------------------------
// Detailed Internal Benchmark (Defeats Compiler Shortcuts)
// -----------------------------------------------------------------------------

static mp_obj_t experimental_benchmark_detailed(mp_obj_t n_in) {
    int32_t n = mp_obj_get_int(n_in);
    volatile float result = 0.0f; 
    uint32_t t0, t1, t2, t3;
    float inv_n = 1.0f / (float)n;
    
    t0 = mp_hal_ticks_ms();
    for (int32_t i = 0; i < n; i++) {
        // Varying input prevents the compiler from pre-calculating
        result += fast_sin_internal((float)i * inv_n);
    }
    
    t1 = mp_hal_ticks_ms();
    for (int32_t i = 0; i < n; i++) {
        result += fast_sin_internal(((float)i * inv_n) + HALF_PI_F);
    }

    t2 = mp_hal_ticks_ms();
    for (int32_t i = 0; i < n; i++) {
        // Changing y and x forces different branches in atan2
        result += fast_atan2_internal((float)i, (float)(n - i));
    }
    t3 = mp_hal_ticks_ms();

    mp_obj_t tuple[4] = {
        mp_obj_new_int(t1 - t0), 
        mp_obj_new_int(t2 - t1), 
        mp_obj_new_int(t3 - t2), 
        mp_obj_new_int(t3 - t0)  
    };
    return mp_obj_new_tuple(4, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_benchmark_detailed_obj, experimental_benchmark_detailed);

static const mp_rom_map_elem_t experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),             MP_ROM_QSTR(MP_QSTR_experimental) },
    { MP_ROM_QSTR(MP_QSTR_sin),                  MP_ROM_PTR(&experimental_sin_obj) },
    { MP_ROM_QSTR(MP_QSTR_cos),                  MP_ROM_PTR(&experimental_cos_obj) },
    { MP_ROM_QSTR(MP_QSTR_atan2),                MP_ROM_PTR(&experimental_atan2_obj) },
    { MP_ROM_QSTR(MP_QSTR_benchmark_detailed),   MP_ROM_PTR(&experimental_benchmark_detailed_obj) },
};
static MP_DEFINE_CONST_DICT(pb_module_experimental_globals, experimental_globals_table);

const mp_obj_module_t pb_module_experimental = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pb_module_experimental_globals,
};

#if !MICROPY_MODULE_BUILTIN_SUBPACKAGES
MP_REGISTER_MODULE(MP_QSTR_pybricks_dot_experimental, pb_module_experimental);
#endif
#endif