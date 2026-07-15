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
#include "arm_math.h"

void Home_Stepper_Motor(Stepper_t *motor, uint8_t motor_id, uint8_t search_forward);
void Robot_Init_State(float32_t init_Angle[4]); // Updated signature

void Robot_Set_IK_Target(const float32_t T_d[16]);
void Robot_Set_IK_Target_PD(const float32_t T_d[16], uint8_t param_idx);
void Robot_IK_Stop(void);

void Robot_Set_Joint_Target(uint8_t motor_id, uint8_t dir, float32_t target_deg);
void Robot_Joint_Task_Service(void);

void Robot_Start_Multi_IK(void);
void Robot_Multi_IK_Service(void);

#endif /* INC_ROBOT_CONTROL_H_ */
