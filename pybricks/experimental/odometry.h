#ifndef PYBRICKS_EXPERIMENTAL_ODOMETRY_H
#define PYBRICKS_EXPERIMENTAL_ODOMETRY_H

#include "py/obj.h"
#include <pbio/servo.h>
#include <stdbool.h>
#include <stdint.h>

// Optimized for Cortex-M4F Alignment
typedef struct {
    pbio_servo_t *left_servo;
    pbio_servo_t *right_servo;

    volatile uint32_t last_time_ms;
    volatile uint32_t mstowait;

    volatile float global_x;
    volatile float global_y;
    volatile float global_h;

    volatile int32_t last_left_angle;
    volatile int32_t last_right_angle;
    volatile float last_imu_heading;

    float deg_to_mm;
    float inv_track;

    volatile uint32_t current_fps;
    volatile uint32_t vm_loop_counter;
    volatile uint32_t last_fps_time_ms;

    volatile bool running;
} pb_odom_state_t;

extern pb_odom_state_t odom_state;

// Background update hook
void pb_background_odometry_update(void);

#endif // PYBRICKS_EXPERIMENTAL_ODOMETRY_H
