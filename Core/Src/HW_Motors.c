/*
 * HW_Motors.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#include "HW_Motors.h"
#include "Custom_CLI.h" // For UART_Printf
#include "Custom_utils.h" // For delay_us, micros
#include "HW_Encoders.h" // For Encoders_Update
#include "HW_LimitSwitches.h" // For limit switches checking
#include <string.h>
#include <stdio.h>
void DC_Stop(void)
{
    g_dc_running = 0;

    HAL_GPIO_WritePin(DC_AIN1_PORT, DC_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC_AIN2_PORT, DC_AIN2_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);

    UART_Printf("M4 DC STOP\r\n");
}

void DC_Start(uint8_t forward)
{
	#if STOP_ON_LIMIT
		if (Limit4_IsTriggered())
		{
			UART_Printf("Cannot start M4: LIMIT4 triggered\r\n");
			return;
		}
	#endif

	g_dc_running = 1;
	g_dc_forward = forward;

	HAL_GPIO_WritePin(DC_STBY_PORT, DC_STBY_PIN, GPIO_PIN_SET);

	if (forward)
	{
		HAL_GPIO_WritePin(DC_AIN1_PORT, DC_AIN1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DC_AIN2_PORT, DC_AIN2_PIN, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(DC_AIN1_PORT, DC_AIN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DC_AIN2_PORT, DC_AIN2_PIN, GPIO_PIN_SET);
	}

	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 700);

	UART_Printf("M4 DC START %s PWM=%u\r\n",
				forward ? "FORWARD" : "REVERSE",
				g_dc_pwm);
}

void Stepper_Enable(Stepper_t *m)
{
    HAL_GPIO_WritePin(m->enPort, m->enPin, m->enRunLevel);
}

void Stepper_Disable(Stepper_t *m)
{
    HAL_GPIO_WritePin(m->enPort, m->enPin, Invert_PinState(m->enRunLevel));
}

void Stepper_SetDir(Stepper_t *m, uint8_t forward)
{
    GPIO_PinState level;

    if (forward)
    {
        level = m->dirForwardLevel;
    }
    else
    {
        level = Invert_PinState(m->dirForwardLevel);
    }

    HAL_GPIO_WritePin(m->dirPort, m->dirPin, level);
}

void Stepper_Stop(Stepper_t *m)
{
    m->running = 0;
    m->stepHigh = 0;

    HAL_GPIO_WritePin(m->stepPort, m->stepPin, GPIO_PIN_RESET);
    Stepper_Disable(m);

    UART_Printf("%s STOP\r\n", m->name);
}

void Stepper_Start(Stepper_t *m, uint8_t motor_id, uint8_t forward)
{
#if STOP_ON_LIMIT
    if (Motor_Limit_IsTriggered(motor_id))
    {
        UART_Printf("Cannot start M%d: LIMIT%d triggered\r\n", motor_id, motor_id);
        return;
    }
#endif

    // FIX: If the motor is already running in the requested direction, do nothing.
    // This prevents the acceleration profile and timers from being abruptly reset.
    if (m->running && m->forward == forward)
    {
        return;
    }

    Stepper_SetDir(m, forward);
    Stepper_Enable(m);

    m->running = 1;
    m->forward = forward;
    m->stepHigh = 0;
    m->currentIntervalUs = START_INTERVAL_US;
    m->lastStepUs = micros();
    m->highStartUs = micros();

    UART_Printf("%s START %s\r\n", m->name, forward ? "FORWARD" : "REVERSE");
}

void Stepper_Service(Stepper_t *m)
{
    if (!m->running)
    {
        return;
    }

    uint32_t now = micros();

    if (m->stepHigh)
    {
        if ((uint32_t)(now - m->highStartUs) >= STEP_HIGH_US)
        {
            HAL_GPIO_WritePin(m->stepPort, m->stepPin, GPIO_PIN_RESET);
            m->stepHigh = 0;
        }

        return;
    }

    if ((uint32_t)(now - m->lastStepUs) >= m->currentIntervalUs)
    {
        HAL_GPIO_WritePin(m->stepPort, m->stepPin, GPIO_PIN_SET);

        m->stepHigh = 1;
        m->highStartUs = now;
        m->lastStepUs = now;

        if (m->currentIntervalUs > m->targetIntervalUs)
		{
			if (m->currentIntervalUs > (m->targetIntervalUs + ACCEL_STEP_US))
			{
				m->currentIntervalUs -= ACCEL_STEP_US;
			}
			else
			{
				m->currentIntervalUs = m->targetIntervalUs;
			}
		}
    }
}

void Stepper_All_Stop(void)
{
    Stepper_Stop(&motor1);
    Stepper_Stop(&motor2);
    Stepper_Stop(&motor4);
}

void Run_Motor_Fixed_Time_Blocking(Stepper_t* motor, uint32_t pulses, uint8_t direction, float time_seconds) {
    char msg[128];

    // Prevent divide-by-zero or negative time errors
    if (pulses == 0 || time_seconds <= 0.0f) {
        return;
    }

    // 1. Calculate the required delay per pulse in microseconds
    uint32_t total_time_us = (uint32_t)(time_seconds * 1000000.0f);
    uint32_t delay_per_pulse_us = total_time_us / pulses;

    // 2. Set direction and enable motor
    GPIO_PinState dirState;
    if (direction == 1) {
        dirState = motor->dirForwardLevel;
    } else {
        dirState = (motor->dirForwardLevel == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, dirState);
    HAL_GPIO_WritePin(motor->enPort, motor->enPin, motor->enRunLevel);

    sprintf(msg, "Running %s: %lu pulses in %.2f seconds...\r\n", motor->name, pulses, time_seconds);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    sprintf(msg, "Calculated delay: %lu us/pulse\r\n", delay_per_pulse_us);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    // Force an encoder update before moving
    Encoders_Update();

    // 3. Send exact number of pulses
    for (uint32_t i = 0; i < pulses; i++) {
        HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, GPIO_PIN_SET);
        delay_us(STEP_HIGH_US);
        HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, GPIO_PIN_RESET);

        // Subtract the pulse high time from the total interval to maintain exact overall timing
        if (delay_per_pulse_us > STEP_HIGH_US) {
            delay_us(delay_per_pulse_us - STEP_HIGH_US);
        }

        // Update periodically to prevent 16-bit timer overflow (TIM3/TIM4)
        if (i % 100 == 0) {
            Encoders_Update();
        }
    }

    // Force a final update after settling
    HAL_Delay(100);
    Encoders_Update();

    sprintf(msg, "%s finished move.\r\n", motor->name);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}
