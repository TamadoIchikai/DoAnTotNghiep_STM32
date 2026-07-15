/*
 * Controllers.c
 *
 *  Created on: Jul 15, 2026
 *      Author: User
 */
#include "Controllers.h"
#include <math.h>

// Stepper PD calculation
// Returns targetIntervalUs, sets *dir based on sign of u.
uint32_t PD_Stepper_Compute(float Kp, float Kd, float error, float *prev_error, float *prev_u, uint32_t pulses_per_rev, uint8_t *dir) {
    float de = error - *prev_error;
    float u = Kp * error + Kd * de;
    *prev_error = error;
    
    // Acceleration Limiter (Slew Rate)
    // Max change in u per loop iteration (assuming ~10ms loop time)
    // 100 degrees/sec max change per iteration = 10,000 deg/sec^2
    float max_delta_u = 50.0f; // Adjust this value to tune acceleration
    
    if (u > *prev_u + max_delta_u) {
        u = *prev_u + max_delta_u;
    } else if (u < *prev_u - max_delta_u) {
        u = *prev_u - max_delta_u;
    }
    
    *prev_u = u;

    if (u > 0) {
        *dir = 1;
    } else {
        *dir = 0;
    }
    
    float abs_u = fabsf(u);
    if (abs_u < 1.0f) { // If velocity is extremely low, just stop
        return 0; // 0 means stopped
    }
    
    // Convert velocity u [deg/s] to interval [us]
    // f = abs_u * (PPR / 360) Hz
    // interval = 1,000,000 / f = (360 * 1,000,000) / (abs_u * PPR)
    float interval = (360.0f * 1000000.0f) / (abs_u * (float)pulses_per_rev);
    
    if (interval > 10000.0f) return 10000;
    if (interval < 200.0f) return 200; // Max speed limit (200us per step = 5kHz)
    
    return (uint32_t)interval;
}

// DC Motor PD calculation
// Returns PWM (0-1000), sets *dir
uint16_t PD_DC_Compute(float Kp, float Kd, float error, float *prev_error, uint8_t *dir) {
    float de = error - *prev_error;
    float u = Kp * error + Kd * de;
    *prev_error = error;
    
    if (u > 0) {
        *dir = 1;
    } else {
        *dir = 0;
    }
    
    float abs_u = fabsf(u);
    if (abs_u > 255.0f) abs_u = 255.0f; // clamp to max PWM
    
    return (uint16_t)abs_u;
}


