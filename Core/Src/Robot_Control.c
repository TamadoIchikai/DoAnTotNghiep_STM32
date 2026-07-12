/*
 * Robot_Control.c
 */

#include "Robot_Control.h"
#include "Custom_CLI.h"       // For UART_Printf
#include "HW_LimitSwitches.h" // For Motor_Limit_IsTriggered
#include "HW_Encoders.h"      // For Encoders_Reset, Encoder_To_Angle
#include "Custom_utils.h"     // For delay_us, micros
#include <stdio.h>
#include <string.h>
#include <math.h>

void Home_Stepper_Motor(Stepper_t *motor, uint8_t motor_id, uint8_t search_forward)
{
    char msg[64];
    sprintf(msg, "Homing %s...\r\n", motor->name);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    // 1. Set search direction and enable motor
    Stepper_SetDir(motor, search_forward);
    Stepper_Enable(motor);

    // 2. Step until limit switch is triggered
    while (!Motor_Limit_IsTriggered(motor_id))
    {
        HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, GPIO_PIN_SET);
        delay_us(STEP_HIGH_US);
        HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, GPIO_PIN_RESET);
        delay_us(motor->targetIntervalUs);
    }

    // 3. Reverse direction to back off
    uint8_t backoff_forward = search_forward ? 0 : 1;
    Stepper_SetDir(motor, backoff_forward);

    // 4. Step until the limit switch is completely released
    while (Motor_Limit_IsTriggered(motor_id))
    {
        HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, GPIO_PIN_SET);
        delay_us(STEP_HIGH_US);
        HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, GPIO_PIN_RESET);
        delay_us(motor->targetIntervalUs);
    }

    // 5. Ensure motor completely stops
    Stepper_Stop(motor);

    sprintf(msg, "%s homing complete.\r\n", motor->name);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

// Main parallel initialization routine using encoder feedback for offset tracking
void Robot_Init_State(float init_Angle[4])
{
    UART_Printf("--- STARTING PARALLEL ROBOT INITIALIZATION ---\r\n");

    Stepper_t* m_ptrs[3] = {&motor1, &motor2, &motor4};
    uint8_t m_ids[3] = {1, 2, 4};

    // Pointers to the globally updated encoder variables
    volatile int32_t* enc_ptrs[3] = {&enc1_pos, &enc2_pos, &enc4_pos};

    // Original search directions: M1: CCW (1), M2: CW (0), M4: CCW (1)
    uint8_t search_dirs[3] = {1, 0, 1};

    float angles[3] = {init_Angle[0], init_Angle[1], init_Angle[3]};

    uint8_t state[3] = {0, 0, 0}; // 0:Search, 1:Backoff, 2:Offset, 3:Done
    uint8_t target_dirs[3];
    int32_t zero_counts[3] = {0, 0, 0};
    uint32_t last_us[3];

    for (int i = 0; i < 3; i++)
    {
        Stepper_SetDir(m_ptrs[i], search_dirs[i]);
        Stepper_Enable(m_ptrs[i]);

        // 1 maps to positive angle ('t' direction), 0 maps to negative angle ('n' direction)
        target_dirs[i] = (angles[i] >= 0.0f) ? 1 : 0;
        last_us[i] = micros();
    }

    uint8_t all_done = 0;

    // Parallel state machine logic
    while (!all_done)
    {
        uint32_t now = micros();
        all_done = 1;

        // Ensure encoders continuously update so counts aren't missed while stepping
        Encoders_Update();

        for (int i = 0; i < 3; i++)
        {
            if (state[i] < 3)
            {
                all_done = 0; // If any motor is not done, keep looping

                if ((uint32_t)(now - last_us[i]) >= m_ptrs[i]->targetIntervalUs)
                {
                    last_us[i] = now;

                    if (state[i] == 0) // SEARCH PHASE
                    {
                        if (Motor_Limit_IsTriggered(m_ids[i]))
                        {
                            state[i] = 1;
                            Stepper_SetDir(m_ptrs[i], search_dirs[i] ? 0 : 1); // Reverse direction for backoff
                        }
                        else
                        {
                            HAL_GPIO_WritePin(m_ptrs[i]->stepPort, m_ptrs[i]->stepPin, GPIO_PIN_SET);
                            delay_us(STEP_HIGH_US);
                            HAL_GPIO_WritePin(m_ptrs[i]->stepPort, m_ptrs[i]->stepPin, GPIO_PIN_RESET);
                        }
                    }
                    else if (state[i] == 1) // BACKOFF PHASE
                    {
                        if (!Motor_Limit_IsTriggered(m_ids[i]))
                        {
                            state[i] = 2;
                            Stepper_SetDir(m_ptrs[i], target_dirs[i]); // Apply target offset direction
                            zero_counts[i] = *enc_ptrs[i];             // Record true physical zero position

                            if (fabsf(angles[i]) < 0.1f)
                            {
                                state[i] = 3; // Skip offset phase if ~0 degrees requested
                                Stepper_Stop(m_ptrs[i]);
                            }
                        }
                        else
                        {
                            HAL_GPIO_WritePin(m_ptrs[i]->stepPort, m_ptrs[i]->stepPin, GPIO_PIN_SET);
                            delay_us(STEP_HIGH_US);
                            HAL_GPIO_WritePin(m_ptrs[i]->stepPort, m_ptrs[i]->stepPin, GPIO_PIN_RESET);
                        }
                    }
                    else if (state[i] == 2) // OFFSET PHASE
                    {
                        // Calculate magnitude of angular change relative to the limit release point
                        int32_t diff = *enc_ptrs[i] - zero_counts[i];
                        float current_angle_diff = fabsf(Encoder_To_Angle(diff, ENCODER_PPR, UNIT_DEG));

                        if (current_angle_diff < fabsf(angles[i]))
                        {
                            HAL_GPIO_WritePin(m_ptrs[i]->stepPort, m_ptrs[i]->stepPin, GPIO_PIN_SET);
                            delay_us(STEP_HIGH_US);
                            HAL_GPIO_WritePin(m_ptrs[i]->stepPort, m_ptrs[i]->stepPin, GPIO_PIN_RESET);
                        }
                        else
                        {
                            state[i] = 3;
                            Stepper_Stop(m_ptrs[i]); // Turn off and flag as complete
                        }
                    }
                }
            }
        }
    }

    // Establish the final offset resting position as the operational angle zero
    Encoders_Reset();
    UART_Printf("--- PARALLEL INITIALIZATION COMPLETE ---\r\n");
}
