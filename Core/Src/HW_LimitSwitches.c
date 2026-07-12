/*
 * HW_LimitSwitches.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#include "HW_LimitSwitches.h"
#include "Custom_CLI.h" // For UART_Printf
#include "HW_Motors.h"  // For motor1, motor2, Stepper_Stop, DC_Stop

uint8_t Limit1_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM1_PORT, LIM1_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit2_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM2_PORT, LIM2_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit3_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM3_PORT, LIM3_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit4_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM4_PORT, LIM4_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit5_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM5_PORT, LIM5_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit6_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM6_PORT, LIM6_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Motor_Limit_IsTriggered(uint8_t motor_id)
{
    if (motor_id == 1) return Limit1_IsTriggered();
    if (motor_id == 2) return Limit2_IsTriggered();
    if (motor_id == 3) return Limit3_IsTriggered();
    if (motor_id == 4) return Limit4_IsTriggered();

    return 0;
}

void Limit_Safety_Service(void)
{
	#if STOP_ON_LIMIT
		if (motor1.running && Limit1_IsTriggered())
		{
			UART_Printf("LIMIT1 triggered -> stop M1\r\n");
			Stepper_Stop(&motor1);
		}

		if (motor2.running && Limit2_IsTriggered())
		{
			UART_Printf("LIMIT2 triggered -> stop M2\r\n");
			Stepper_Stop(&motor2);
		}

		if (g_dc_running && Limit3_IsTriggered())
		{
			UART_Printf("LIMIT3 triggered -> stop M3 DC\r\n");
			DC_Stop();
		}

		if (motor4.running && Limit4_IsTriggered())
		{
			UART_Printf("LIMIT4 triggered -> stop M4\r\n");
			Stepper_Stop(&motor4);
		}
	#endif
}
