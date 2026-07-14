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
    static float m1_jam_start = 0, m2_jam_start = 0, m4_jam_start = 0;
    static uint8_t m1_jammed = 0, m2_jammed = 0, m4_jammed = 0;
    static uint8_t m1_opp_track = 0, m2_opp_track = 0, m4_opp_track = 0;
    
    static uint8_t l1_prev = 0, l2_prev = 0, l3_prev = 0, l4_prev = 0;
    uint8_t l1_curr = Limit1_IsTriggered();
    uint8_t l2_curr = Limit2_IsTriggered();
    uint8_t l3_curr = Limit3_IsTriggered();
    uint8_t l4_curr = Limit4_IsTriggered();

    // Motor 1: CCW limit (forward=1), reverses CW (forward=0)
    if (l1_curr) {
        if (m1_jammed) { /* Do nothing until released */ }
        else if (!motor1.running || motor1.forward == 1) {
            if (!l1_prev) UART_Printf("LIMIT1 triggered -> backing off M1\r\n");
            Stepper_Start(&motor1, 0, 0); // Reverse
            m1_opp_track = 1;
            m1_jam_start = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG);
        } else if (motor1.running && motor1.forward == 0) {
            if (!m1_opp_track) { m1_opp_track = 1; m1_jam_start = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG); }
            else if (fabsf(Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG) - m1_jam_start) >= 2.0f) {
                if (!l1_prev) UART_Printf("[ERR] M1 Limit Jammed! Stopped after 5deg\r\n");
                Stepper_Stop(&motor1);
                m1_opp_track = 0; m1_jammed = 1;
            }
        }
    } else {
        if (m1_opp_track) { Stepper_Stop(&motor1); m1_opp_track = 0; }
        m1_jammed = 0;
    }

    // Motor 2: CW limit (forward=0), reverses CCW (forward=1)
    if (l2_curr) {
        if (m2_jammed) { /* Do nothing */ }
        else if (!motor2.running || motor2.forward == 0) {
            if (!l2_prev) UART_Printf("LIMIT2 triggered -> backing off M2\r\n");
            Stepper_Start(&motor2, 0, 1);
            m2_opp_track = 1;
            m2_jam_start = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG);
        } else if (motor2.running && motor2.forward == 1) {
            if (!m2_opp_track) { m2_opp_track = 1; m2_jam_start = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG); }
            else if (fabsf(Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG) - m2_jam_start) >= 2.0f) {
                if (!l2_prev) UART_Printf("[ERR] M2 Limit Jammed! Stopped after 5deg\r\n");
                Stepper_Stop(&motor2);
                m2_opp_track = 0; m2_jammed = 1;
            }
        }
    } else {
        if (m2_opp_track) { Stepper_Stop(&motor2); m2_opp_track = 0; }
        m2_jammed = 0;
    }

    // Motor 3 DC motor logic
    if (g_dc_running && l3_curr) {
        if (!l3_prev) UART_Printf("LIMIT3 triggered -> stop M3 DC\r\n");
        DC_Stop();
    }

    // Motor 4: CCW limit (forward=1), reverses CW (forward=0)
    if (l4_curr) {
        if (m4_jammed) { /* Do nothing */ }
        else if (!motor4.running || motor4.forward == 1) {
            if (!l4_prev) UART_Printf("LIMIT4 triggered -> backing off M4\r\n");
            Stepper_Start(&motor4, 0, 0);
            m4_opp_track = 1;
            m4_jam_start = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG);
        } else if (motor4.running && motor4.forward == 0) {
            if (!m4_opp_track) { m4_opp_track = 1; m4_jam_start = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG); }
            else if (fabsf(Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG) - m4_jam_start) >= 2.0f) {
                if (!l4_prev) UART_Printf("[ERR] M4 Limit Jammed! Stopped after 5deg\r\n");
                Stepper_Stop(&motor4);
                m4_opp_track = 0; m4_jammed = 1;
            }
        }
    } else {
        if (m4_opp_track) { Stepper_Stop(&motor4); m4_opp_track = 0; }
        m4_jammed = 0;
    }
    
    l1_prev = l1_curr; l2_prev = l2_curr; l3_prev = l3_curr; l4_prev = l4_curr;
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
