#ifndef PYBRICKS_EXPERIMENTAL_PURSUIT_H
#define PYBRICKS_EXPERIMENTAL_PURSUIT_H

#include "py/obj.h"
#include <stdbool.h>
#include <stdint.h>

// pure pursuit state struct (Optimized for Cortex-M4F)
typedef struct {
    float (*spline_coefficients)[8];
    volatile uint32_t last_time_ms;

    volatile float t_lookahead;
    volatile float lookahead;
    volatile float target_x;
    volatile float target_y;

    volatile float left_motor_speed;
    volatile float right_motor_speed;

    float max_speed;
    float min_speed;
    float base_speed_percentage;
    float max_per_motor_acceleration;
    float min_lookahead;
    float max_lookahead;
    float min_curvature;
    float max_curvature;

    uint16_t total_splines;
    uint8_t total_newton_iterations;

    volatile bool running;
} pb_pursuit_state_t;

extern pb_pursuit_state_t pursuit_state;

void pb_background_pursuit_update(void);

#endif // PYBRICKS_EXPERIMENTAL_PURSUIT_H
