/*
 * HW_LimitSwitches.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_HW_LIMITSWITCHES_H_
#define INC_HW_LIMITSWITCHES_H_

#include <stdint.h>
#include "main.h"
#include <math.h>

extern float M1_Lim_Deg;
extern float M2_Lim_Deg;
extern float M4_Lim_mm;
extern float M4_Pulley_Linear_Dia;

uint8_t Limit1_IsTriggered(void);
uint8_t Limit2_IsTriggered(void);
uint8_t Limit3_IsTriggered(void);
uint8_t Limit4_IsTriggered(void);
uint8_t Limit5_IsTriggered(void);
uint8_t Limit6_IsTriggered(void);
uint8_t Motor_Limit_IsTriggered(uint8_t motor_id);
void Limit_Safety_Service(void);
void Software_Limit_Service(void);
#endif /* INC_HW_LIMITSWITCHES_H_ */
