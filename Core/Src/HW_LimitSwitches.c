/*
 * HW_LimitSwitches.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#include "HW_LimitSwitches.h"
#include "Custom_CLI.h" // For UART_Printf
#include "HW_Motors.h"  // For motor1, motor2, Stepper_Stop, DC_Stop
#include "HW_Encoders.h" // For enc1_pos, Encoder_To_Angle, Angle_To_Distance
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
    static uint8_t m1_recovering = 0;
    static uint8_t m2_recovering = 0;
    static uint8_t m4_recovering = 0;

    // Motor 1: pressed on CCW (forward=1), release by moving CW (forward=0)
    if (Limit1_IsTriggered()) {
        if (!motor1.running || motor1.forward == 1) {
            UART_Printf("LIMIT1 triggered -> reversing M1\r\n");
            Stepper_Start(&motor1, 0, 0); // motor_id=0 bypasses limit check
            m1_recovering = 1;
        }
    } else if (m1_recovering) {
        Stepper_Stop(&motor1);
        m1_recovering = 0;
    }

    // Motor 2: pressed on CW (forward=0), release by moving CCW (forward=1)
    if (Limit2_IsTriggered()) {
        if (!motor2.running || motor2.forward == 0) {
            UART_Printf("LIMIT2 triggered -> reversing M2\r\n");
            Stepper_Start(&motor2, 0, 1);
            m2_recovering = 1;
        }
    } else if (m2_recovering) {
        Stepper_Stop(&motor2);
        m2_recovering = 0;
    }

    // Motor 3 DC motor logic unchanged
    if (g_dc_running && Limit3_IsTriggered()) {
        UART_Printf("LIMIT3 triggered -> stop M3 DC\r\n");
        DC_Stop();
    }

    // Motor 4: pressed on CCW (forward=1), release by moving CW (forward=0)
    if (Limit4_IsTriggered()) {
        if (!motor4.running || motor4.forward == 1) {
            UART_Printf("LIMIT4 triggered -> reversing M4\r\n");
            Stepper_Start(&motor4, 0, 0);
            m4_recovering = 1;
        }
    } else if (m4_recovering) {
        Stepper_Stop(&motor4);
        m4_recovering = 0;
    }
#endif
}

void Software_Limit_Service(void)
{
#if STOP_ON_LIMIT
    // ---------------------------------------------------------
    // MOTOR 1 SOFT LIMIT: 180 degrees
    // Homes CCW (forward = 1). Moves away from home CW (forward = 0).
    // ---------------------------------------------------------
    float m1_angle = fabsf(Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG));
    if (m1_angle >= M1_Lim_Deg) {
        // Only stop if it is actively trying to move further away (CW)
        if (motor1.running && motor1.forward == 0) {
            UART_Printf("SOFT LIMIT: M1 exceeded 180 deg. Stopping.\r\n");
            Stepper_Stop(&motor1);
        }
    }

    // ---------------------------------------------------------
    // MOTOR 2 SOFT LIMIT: 200 degrees
    // Homes CW (forward = 0). Moves away from home CCW (forward = 1).
    // ---------------------------------------------------------
    float m2_angle = fabsf(Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG));
    if (m2_angle >= M2_Lim_Deg) {
        // Only stop if it is actively trying to move further away (CCW)
        if (motor2.running && motor2.forward == 1) {
            UART_Printf("SOFT LIMIT: M2 exceeded 200 deg. Stopping.\r\n");
            Stepper_Stop(&motor2);
        }
    }

    // ---------------------------------------------------------
    // MOTOR 4 SOFT LIMIT: 95 mm (Linear Prismatic)
    // Homes CCW (forward = 1). Moves away from home CW (forward = 0).
    // ---------------------------------------------------------
    float m4_angle = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG);
    // Pulley diameter = 14mm
    float m4_dist = fabsf(Angle_To_Distance(m4_angle, M4_Pulley_Linear_Dia, UNIT_DEG));

    if (m4_dist >= M4_Lim_mm) {
        // Only stop if it is actively trying to move further away (CW)
        if (motor4.running && motor4.forward == 0) {
            UART_Printf("SOFT LIMIT: M4 exceeded 95 mm. Stopping.\r\n");
            Stepper_Stop(&motor4);
        }
    }
#endif
}
