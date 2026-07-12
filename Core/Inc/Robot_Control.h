/*
 * Robot_Control.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_

#include <stdint.h>
#include "main.h"
#include "HW_Motors.h"

void Home_Stepper_Motor(Stepper_t *motor, uint8_t motor_id, uint8_t search_forward);
void Robot_Init_State(float init_Angle[4]); // Updated signature

#endif /* INC_ROBOT_CONTROL_H_ */
