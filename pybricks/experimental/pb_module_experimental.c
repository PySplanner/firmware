// SPDX-License-Identifier: MIT
#include "py/mpconfig.h"
#include "py/obj.h"
#include "py/runtime.h"

#if PYBRICKS_PY_EXPERIMENTAL

// Forward declarations
extern mp_obj_t experimental_start_odometry(size_t n_args, const mp_obj_t *args);
extern mp_obj_t experimental_get_odometry(void);
extern mp_obj_t experimental_stop_odometry(void);
extern mp_obj_t experimental_start_pursuit(size_t n_args, const mp_obj_t *args);
extern mp_obj_t experimental_stop_pursuit(void);
extern mp_obj_t experimental_get_fps(void);

// Object definitions
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(experimental_start_odometry_obj, 8, 8, experimental_start_odometry);
static MP_DEFINE_CONST_FUN_OBJ_0(experimental_get_odometry_obj, experimental_get_odometry);
static MP_DEFINE_CONST_FUN_OBJ_0(experimental_stop_odometry_obj, experimental_stop_odometry);

// FIXED: Pursuit takes 7 arguments (a, b, c, d, x_end, speed, lookahead), not 4.
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(experimental_start_pursuit_obj, 7, 7, experimental_start_pursuit);

static MP_DEFINE_CONST_FUN_OBJ_0(experimental_stop_pursuit_obj, experimental_stop_pursuit);
static MP_DEFINE_CONST_FUN_OBJ_0(experimental_get_fps_obj, experimental_get_fps);

const mp_rom_map_elem_t pb_module_experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_experimental) },
    { MP_ROM_QSTR(MP_QSTR_start_odometry),    MP_ROM_PTR(&experimental_start_odometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_odometry),      MP_ROM_PTR(&experimental_get_odometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_odometry),     MP_ROM_PTR(&experimental_stop_odometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_pursuit),     MP_ROM_PTR(&experimental_start_pursuit_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_pursuit),      MP_ROM_PTR(&experimental_stop_pursuit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fps),           MP_ROM_PTR(&experimental_get_fps_obj) },
};
MP_DEFINE_CONST_DICT(pb_module_experimental_globals, pb_module_experimental_globals_table);

const mp_obj_module_t pb_module_experimental = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pb_module_experimental_globals,
};

#endif
