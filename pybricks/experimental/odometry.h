#ifndef PYBRICKS_EXPERIMENTAL_ODOMETRY_H
#define PYBRICKS_EXPERIMENTAL_ODOMETRY_H

#include "py/obj.h"

// The signature must use float to match the core and avoid double-promotion errors
mp_obj_t calculate_odometry(int num_iters, float wheel_circ, float axle_track, mp_obj_t right_angle_func, mp_obj_t left_angle_func);

#endif // PYBRICKS_EXPERIMENTAL_ODOMETRY_H