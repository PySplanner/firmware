#include "py/mpconfig.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/builtin.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "pybricks/experimental/odometry.h"

// Define STATIC if not already defined (usually it is in mpconfig.h)
#ifndef STATIC
#define STATIC static
#endif

// This is the function that Python calls
STATIC mp_obj_t experimental_odometry_benchmark(size_t n_args, const mp_obj_t *args) {
    int num_iters = mp_obj_get_int(args[0]);
    float wheel_circ = mp_obj_get_float(args[1]);
    float axle_track = mp_obj_get_float(args[2]);
    mp_obj_t right_func = args[3];
    mp_obj_t left_func = args[4];

    return calculate_odometry(num_iters, wheel_circ, axle_track, right_func, left_func);
}
// Define as a MicroPython function object
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(experimental_odometry_benchmark_obj, 5, 5, experimental_odometry_benchmark);

// Map the C function to the Python name "odometry_benchmark"
STATIC const mp_rom_map_elem_t experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_odometry_benchmark), MP_ROM_PTR(&experimental_odometry_benchmark_obj) },
};
STATIC MP_DEFINE_CONST_DICT(pb_module_experimental_globals, experimental_globals_table);

// Structure for MicroPython to register the module
const mp_obj_module_t pb_module_experimental = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pb_module_experimental_globals,
};

#endif // PYBRICKS_PY_EXPERIMENTAL