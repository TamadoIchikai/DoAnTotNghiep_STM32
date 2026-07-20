/*
 * Controllers.c
 *
 *  Created on: Jul 15, 2026
 *      Author: User
 */
#include "Controllers.h"
#include <math.h>

// Stepper PD calculation
uint32_t PD_Stepper_Compute(float Kp, float Kd, float error, float *prev_error, float *prev_u, uint32_t pulses_per_rev, uint8_t *dir, float min_error) {
	if (fabsf(error) < min_error) // Use min_error for positional deadband
	{
		error = 0.0f;
	}

    float de = error - *prev_error;
    float u = Kp * error + Kd * de;
    *prev_error = error;
    
    // Asymmetric Slew Rate (Acceleration Limiter)
    // We allow faster deceleration than acceleration to prevent overshoot and limit-cycle vibrations!
    float max_accel = 100.0f;
    float max_decel = 100.0f;
    
    // Determine if we are accelerating or decelerating
    // Accelerating: u and prev_u have same sign, and |u| > |prev_u|
    uint8_t is_accel = 0;
    if ((u > 0 && *prev_u >= 0 && u > *prev_u) || 
        (u < 0 && *prev_u <= 0 && u < *prev_u)) {
        is_accel = 1;
    }
    
    float delta_limit = is_accel ? max_accel : max_decel;
    
    if (u > *prev_u + delta_limit) {
        u = *prev_u + delta_limit;
    } else if (u < *prev_u - delta_limit) {
        u = *prev_u - delta_limit;
    }
    
    *prev_u = u;

    if (u > 0) {
        *dir = 1;
    } else {
        *dir = 0;
    }
    
    float abs_u = fabsf(u);
    
    // We MUST have a check to return 0 when u is zero, otherwise division by zero below gives Infinity, 
    // which then gets capped at 10000 (100Hz), meaning the motor never stops!
    if (abs_u < 0.01f) { 
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
uint16_t PD_DC_Compute(float Kp, float Kd, float error, float *prev_error, float *prev_u, uint8_t *dir, float min_error) {
    if (fabsf(error) < min_error) {
        error = 0.0f;
    }

    float de = error - *prev_error;
    float u = Kp * error + Kd * de;
    *prev_error = error;
    
    // Slew rate / Acceleration limiter for DC motor to protect H-bridge
    // Max change in PWM per loop iteration (~10ms loop)
    float max_delta_u = 15.0f; 
    
    if (u > *prev_u + max_delta_u) {
        u = *prev_u + max_delta_u;
    } else if (u < *prev_u - max_delta_u) {
        u = *prev_u - max_delta_u;
    }
    
    // Position deadband to prevent chattering around the exact setpoint
    if (fabsf(error) < min_error && fabsf(u) < 30.0f) {
        u = 0.0f;
    }
    
    *prev_u = u;
    
    if (u > 0) {
        *dir = 1;
    } else {
        *dir = 0;
    }
    
    float abs_u = fabsf(u);
    // Minimum PWM deadband to overcome friction
    if (abs_u < 10.0f) {
        return 0;
    }
    if (abs_u > 255.0f) abs_u = 255.0f; // clamp to max PWM
    
    return (uint16_t)abs_u;
}


