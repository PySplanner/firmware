#include "py/mpconfig.h"
#include "py/obj.h"
#include "py/runtime.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "pybricks/experimental/odometry.h"

// 1. The actual benchmark logic
// Keep this STATIC as it is used by the function object below
static mp_obj_t experimental_odometry_benchmark(size_t n_args, const mp_obj_t *args) {
    int num_iters = mp_obj_get_int(args[0]);
    float wheel_circ = mp_obj_get_float(args[1]);
    float axle_track = mp_obj_get_float(args[2]);
    mp_obj_t right_func = args[3];
    mp_obj_t left_func = args[4];

    return calculate_odometry(num_iters, wheel_circ, axle_track, right_func, left_func);
}
// Define the function object
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(experimental_odometry_benchmark_obj, 5, 5, experimental_odometry_benchmark);

// 2. The Globals Table
// REMOVED 'STATIC' to prevent 'defined but not used' error
const mp_rom_map_elem_t pb_module_experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_odometry_benchmark), MP_ROM_PTR(&experimental_odometry_benchmark_obj) },
};

// 3. Define the dict WITHOUT 'STATIC'
// This makes it visible to the auto-registration system without triggering the warning
MP_DEFINE_CONST_DICT(pb_module_experimental_globals, pb_module_experimental_globals_table);

// 4. Define the module structure WITHOUT 'STATIC'
// This matches the 'extern' declaration the build system generates
const mp_obj_module_t pb_module_experimental = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pb_module_experimental_globals,
};

#endif // PYBRICKS_PY_EXPERIMENTAL