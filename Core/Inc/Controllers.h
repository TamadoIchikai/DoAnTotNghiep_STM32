/*
 * Controllers.h
 *
 *  Created on: Jul 15, 2026
 *      Author: User
 */

#ifndef INC_CONTROLLERS_H_
#define INC_CONTROLLERS_H_

#include <stdint.h>
#include "main.h"

uint32_t PD_Stepper_Compute(float Kp, float Kd, float error, float *prev_error, float *prev_u, uint32_t pulses_per_rev, uint8_t *dir, float min_error);
uint16_t PD_DC_Compute(float Kp, float Kd, float error, float *prev_error, float *prev_u, uint8_t *dir, float min_error);

#endif /* INC_CONTROLLERS_H_ */
