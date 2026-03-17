#include "py/mpconfig.h"
#include "py/obj.h"
#include "py/runtime.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "pybricks/experimental/odometry.h"

#ifndef STATIC
#define STATIC static
#endif

// 1. The actual function logic
STATIC mp_obj_t experimental_odometry_benchmark(size_t n_args, const mp_obj_t *args) {
    int num_iters = mp_obj_get_int(args[0]);
    float wheel_circ = mp_obj_get_float(args[1]);
    float axle_track = mp_obj_get_float(args[2]);
    mp_obj_t right_func = args[3];
    mp_obj_t left_func = args[4];

    return calculate_odometry(num_iters, wheel_circ, axle_track, right_func, left_func);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(experimental_odometry_benchmark_obj, 5, 5, experimental_odometry_benchmark);

// 2. The Globals Table - the system looks for the name "<filename>_globals"
// Since your file is pb_module_experimental.c, it looks for pb_module_experimental_globals
STATIC const mp_rom_map_elem_t pb_module_experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_odometry_benchmark), MP_ROM_PTR(&experimental_odometry_benchmark_obj) },
};
// This is the specific symbol the linker is clashing on - we keep it STATIC here.
STATIC MP_DEFINE_CONST_DICT(pb_module_experimental_globals, pb_module_experimental_globals_table);

#endif // PYBRICKS_PY_EXPERIMENTAL