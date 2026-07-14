/*
 * Custom_CLI.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#include "Custom_CLI.h"
#include "HW_Motors.h" // For Stepper_Start, DC_Start, etc.
#include "HW_Encoders.h" // For Encoders_Reset
#include "HW_EndEffector.h" // For Magnet_On/Off
#include "HW_LimitSwitches.h" // For Limit1_IsTriggered, etc.
#include <string.h> // For strcmp, strlen
#include <stdlib.h> // For atoi
#include <ctype.h>  // For tolower
#include <stdarg.h>
#include <stdio.h>
#include "PoE_Kinematics.h"
#include "Robot_Control.h"

uint8_t e_stop_active = 0;
char rx_cmd[32];
uint8_t rx_index = 0;

extern Stepper_t motor1;
extern Stepper_t motor2;
extern Stepper_t motor4;

extern uint8_t g_dc_running;
extern uint8_t g_dc_forward;

// --- TX Ring Buffer ---
#define TX_BUF_SIZE 512
static volatile uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;

void UART_PutChar(char ch) {
    uint16_t next = (tx_head + 1) % TX_BUF_SIZE;
    if (next == tx_tail) {
        // Buffer full - must drain one char to make room
        while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET) {}
        huart2.Instance->DR = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUF_SIZE;
    }
    tx_buf[tx_head] = (uint8_t)ch;
    tx_head = next;
}

void UART_TX_Service(void) {
    // Drain characters from ring buffer without blocking
    while (tx_head != tx_tail && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)) {
        huart2.Instance->DR = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUF_SIZE;
    }
}

void UART_PutString(const char* str) {
    while (*str) {
        UART_PutChar(*str++);
    }
}

void UART_PutInt(long val) {
    char buf[12];
    int i = 0;
    long v = val;
    if (v < 0) { UART_PutChar('-'); v = -v; }
    if (v == 0) { buf[i++] = '0'; }
    while (v > 0) { buf[i++] = (v % 10) + '0'; v /= 10; }
    while (i > 0) { UART_PutChar(buf[--i]); }
}

void UART_PutFloatFast(float val) {
    int i = (int)val;
    int f = (int)(fabsf(val) * 100.0f) % 100;
    if (val < 0.0f && i == 0) {
        UART_PutChar('-');
    }
    UART_PutInt(i);
    UART_PutChar('.');
    if (f < 10) UART_PutChar('0');
    UART_PutInt(f);
}

extern void PoETest(void);

void Print_Status(void)
{
    UART_PutString("------------------------\r\n");

    UART_PutString("ENC1="); UART_PutInt(enc1_pos);
    UART_PutString(" | ENC2="); UART_PutInt(enc2_pos);
    UART_PutString(" | ENC3="); UART_PutInt(enc3_pos);
    UART_PutString(" | ENC4="); UART_PutInt(enc4_pos);
    UART_PutString("\r\n");

    float ang1 = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG);
    float ang2 = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG);
    float ang3 = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_DEG);
    float ang4 = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG);
    float dist4 = Angle_To_Distance(ang4, M4_Pulley_Linear_Dia, UNIT_DEG);

    UART_PutString("ANG1="); UART_PutFloatFast(ang1);
    UART_PutString(" | ANG2="); UART_PutFloatFast(ang2);
    UART_PutString(" | ANG3="); UART_PutFloatFast(ang3);
    UART_PutString(" | ANG4="); UART_PutFloatFast(ang4);
    UART_PutString(" (DIST="); UART_PutFloatFast(dist4); UART_PutString("mm)\r\n");

    UART_PutString("LIM=");
    UART_PutInt(Limit1_IsTriggered());
    UART_PutInt(Limit2_IsTriggered());
    UART_PutInt(Limit3_IsTriggered());
    UART_PutInt(Limit4_IsTriggered());
    UART_PutInt(Limit5_IsTriggered());
    UART_PutInt(Limit6_IsTriggered());
    UART_PutString("\r\n");

    UART_PutString("M=");
    UART_PutInt(motor1.running);
    UART_PutInt(motor2.running);
    UART_PutInt(g_dc_running);
    UART_PutInt(motor4.running);
    UART_PutString(" | MAG="); UART_PutInt(g_magnet_on);
    UART_PutString(" | US(1,2,4)=");
    UART_PutInt(motor1.targetIntervalUs); UART_PutChar(',');
    UART_PutInt(motor2.targetIntervalUs); UART_PutChar(',');
    UART_PutInt(motor4.targetIntervalUs);
    UART_PutString(" | DC_PWM="); UART_PutInt(g_dc_pwm);
    UART_PutString("\r\n------------------------\r\n");
}

void Print_Help(void)
{
    UART_Printf("\r\n===== COMMAND LIST =====\r\n");
    UART_Printf("t1/t2/t3/t4 : motor quay thuan\r\n");
    UART_Printf("n1/n2/n3/n4 : motor quay nghich\r\n");
    UART_Printf("s1/s2/s3/s4 : dung motor\r\n");
    UART_Printf("x           : dung tat ca motor\r\n");
    UART_Printf("m1          : bat nam cham\r\n");
    UART_Printf("m0          : tat nam cham\r\n");
    UART_Printf("z           : reset encoder\r\n");
    UART_Printf("status      : in trang thai\r\n");
    UART_Printf("v1200       : set M1 speed (200us). Nho hon = nhanh hon\r\n");
    UART_Printf("w700        : set PWM DC 0..999\r\n");
    UART_Printf("help        : hien bang lenh\r\n");
    UART_Printf("========================\r\n");
}

void Process_Command(char *cmd)
{
    while (*cmd == ' ')
    {
        cmd++;
    }

    for (uint8_t i = 0; cmd[i]; i++)
    {
        cmd[i] = (char)tolower((unsigned char)cmd[i]);
    }

    if (strlen(cmd) == 0)
    {
        return;
    }

    if (e_stop_active)
    {
        UART_Printf("ERR: EMERGENCY STOP ACTIVE. TYPE 'xxx' TO DEACTIVATE.\r\n");
        return;
    }

    if (strcmp(cmd, "help") == 0)
    {
        Print_Help();
        return;
    }

    if (strcmp(cmd, "status") == 0)
    {
        Print_Status();
        return;
    }
    if (strcmp(cmd, "testmat") == 0) // <-- Add this block
	{
		PoETest();
		return;
	}

    if (cmd[0] == 'z')
    {
        Encoders_Reset();
        UART_Printf("Encoder reset\r\n");
        return;
    }

    if (strncmp(cmd, "ang", 3) == 0)
    {
        uint8_t motor_id = cmd[3] - '0';
        char dir_char = cmd[4];
        float32_t target = atof(&cmd[5]);
        
        if (motor_id >= 1 && motor_id <= 4 && (dir_char == 'n' || dir_char == 't')) {
            uint8_t dir = (dir_char == 't') ? 1 : 0;
            Robot_Set_Joint_Target(motor_id, dir, target);
            UART_Printf("Moving M%d %s to %.2f deg\r\n", motor_id, dir ? "POS" : "NEG", target);
        } else {
            UART_Printf("Invalid ang command. Format: ang[id][n/t][deg]\r\n");
        }
        return;
    }

    if (strcmp(cmd, "fk") == 0)
    {
        float32_t thetalist[4];
        thetalist[0] = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_RAD);
        thetalist[1] = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_RAD);
        thetalist[2] = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_RAD);
        float32_t m4_deg = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG);
        thetalist[3] = Angle_To_Distance(m4_deg, M4_Pulley_Linear_Dia, UNIT_DEG) / 1000.0f;

        float32_t T_EE[16];
        ForwardKinematics_Space(M_home, S, thetalist, 4, T_EE);

        UART_Printf("Forward Kinematics T_EE (4x4):\r\n");
        for (int i = 0; i < 4; i++) {
            UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n", 
                T_EE[i*4], T_EE[i*4+1], T_EE[i*4+2], T_EE[i*4+3]);
        }
        return;
    }

    if (strcmp(cmd, "re") == 0)
    {
        extern float32_t robot_Init_Angle[4];
        Robot_Init_State(robot_Init_Angle);
        return;
    }

    if (cmd[0] == 'm')
    {
        int val = atoi(&cmd[1]);

        if (val == 1)
        {
            Magnet_On();
        }
        else
        {
            Magnet_Off();
        }

        return;
    }

    if (cmd[0] == 'v')
	{
		int id = cmd[1] - '0';
		int val = atoi(&cmd[2]);

		if (id < 1 || id > 4) {
			 UART_Printf("Format: v<id><us> (e.g. v1200 for M1 at 200us)\r\n");
			 return;
		}

		if (val < 80) val = 80;
		if (val > 3000) val = 3000;

		if (id == 1) motor1.targetIntervalUs = val;
		else if (id == 2) motor2.targetIntervalUs = val;
		else if (id == 4) motor4.targetIntervalUs = val;
		else {
			 UART_Printf("Unknown motor id\r\n");
			 return;
		}

		UART_Printf("M%d target step interval = %d us\r\n", id, val);
		return;
	}

    if (cmd[0] == 'w')
    {
        int val = atoi(&cmd[1]);

        if (val < 0) val = 0;
        if (val > DC_PWM_MAX) val = DC_PWM_MAX;

        g_dc_pwm = (uint16_t)val;

        if (g_dc_running)
        {
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, g_dc_pwm);
        }

        UART_Printf("Set DC PWM = %u\r\n", g_dc_pwm);
        return;
    }
    // runtime<id>_<t|n>1_<pulses>_<seconds>  e.g. runtime1_n1_3200_5 = motor1 reverse 3200 pulses in 5s
    if (strncmp(cmd, "runtime", 7) == 0) {
        int id = 0;
        char dir_ch = 0;
        int dummy = 0;
        uint32_t pulses = 0;
        int seconds = 0;
        // Parse: runtime<id>_<t|n><digit>_<pulses>_<seconds>
        if (sscanf(cmd, "runtime%d_%c%d_%lu_%d", &id, &dir_ch, &dummy, &pulses, &seconds) == 5) {
            uint8_t forward = (dir_ch == 't') ? 1 : 0;
            Stepper_t* mptr = NULL;
            if (id == 1) mptr = &motor1;
            else if (id == 2) mptr = &motor2;
            else if (id == 4) mptr = &motor4;

            if (mptr && seconds > 0 && pulses > 0) {
                UART_Printf("runtime M%d %s %lu pulses in %ds\r\n", id, forward ? "FWD" : "REV", pulses, seconds);
                Run_Motor_Fixed_Time_Blocking(mptr, pulses, forward, (float)seconds);
            } else {
                UART_Printf("Invalid motor id, pulses or time\r\n");
            }
        } else {
            UART_Printf("Format: runtime<id>_<t|n>1_<pulses>_<seconds> (e.g. runtime1_n1_3200_5)\r\n");
        }
        return;
    }

    if (cmd[0] == 'r') {
            // Example format: r<motor_id> <pulses> <time_in_seconds>
            // Example command: "r2 1600 5.0"

            int id;
            uint32_t pulses;
            float time_sec;

            // Parse the string for 3 variables
            if (sscanf(cmd, "r%d_%lu_%f", &id, &pulses, &time_sec) == 3) {
                if (id == 1) Run_Motor_Fixed_Time_Blocking(&motor1, pulses, 1, time_sec);
                else if (id == 2) Run_Motor_Fixed_Time_Blocking(&motor2, pulses, 1, time_sec);
                else if (id == 4) Run_Motor_Fixed_Time_Blocking(&motor4, pulses, 1, time_sec);
                else UART_Printf("Unknown motor id\r\n");
            } else {
                UART_Printf("Invalid format. Use: r<id> <pulses> <time_sec>\r\n");
            }
            return;
        }

    char action = cmd[0];
    int id = atoi(&cmd[1]);

    if (action == 't')
    {
        if (id == 1) Stepper_Start(&motor1, 1, 1);
        else if (id == 2) Stepper_Start(&motor2, 2, 1);
        else if (id == 3) DC_Start(1);
        else if (id == 4) Stepper_Start(&motor4, 3, 1);
        else UART_Printf("Unknown motor id\r\n");

        return;
    }

    if (action == 'n')
    {
        if (id == 1) Stepper_Start(&motor1, 1, 0);
        else if (id == 2) Stepper_Start(&motor2, 2, 0);
        else if (id == 3) DC_Start(0);
        else if (id == 4) Stepper_Start(&motor4, 3, 0);
        else UART_Printf("Unknown motor id\r\n");

        return;
    }

    if (action == 's')
    {
        if (id == 1) Stepper_Stop(&motor1);
        else if (id == 2) Stepper_Stop(&motor2);
        else if (id == 3) DC_Stop();
        else if (id == 4) Stepper_Stop(&motor4);
        else UART_Printf("Unknown motor id\r\n");

        return;
    }

    UART_Printf("Unknown command: %s\r\n", cmd);
}

static uint8_t x_count = 0;

void UART_Command_Service(void)
{
    // Clear overrun error if set (reading SR then DR clears ORE on STM32F4)
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
        volatile uint32_t tmp = huart2.Instance->SR;
        tmp = huart2.Instance->DR;
        (void)tmp;
    }

    uint8_t ch;
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
    {
        ch = (uint8_t)(huart2.Instance->DR & 0xFF);
        // 1. Universal Stop detection
        if (ch == 'x' || ch == 'X') {
            x_count++;
            if (x_count == 1) {
                Stepper_All_Stop();
                DC_Stop();
                Robot_IK_Stop();
                UART_Printf("\r\n[!] SOFT STOP\r\n> ");
            } else if (x_count == 3) {
                if (e_stop_active == 0) {
                    e_stop_active = 1;
                    Stepper_All_Stop();
                    DC_Stop();
                    Robot_IK_Stop();
                    UART_Printf("\r\n[!] EMERGENCY STOP ACTIVATED!\r\n> ");
                } else {
                    e_stop_active = 0;
                    UART_Printf("\r\n[!] EMERGENCY STOP DEACTIVATED!\r\n> ");
                }
                x_count = 0;
            }
            rx_index = 0; // Clear buffer
            continue;
        } else {
            x_count = 0;
        }

        // 2. Process normal text buffer
        if (ch == '\r' || ch == '\n')
        {
            if (rx_index > 0)
            {
                UART_Printf("\r\n");
                rx_cmd[rx_index] = '\0';
                Process_Command(rx_cmd);
            }
            else if (ch == '\r') // avoid double prompt for \r\n
            {
                UART_Printf("\r\n");
            }
            rx_index = 0; // Always clear on Enter
            UART_Printf("> ");
        }
        else if (ch == '\b' || ch == 127) // Backspace
        {
            if (rx_index > 0)
            {
                rx_index--;
                UART_Printf("\b \b");
            }
        }
        else if (rx_index < sizeof(rx_cmd) - 1)
        {
            rx_cmd[rx_index++] = (char)ch;
            UART_PutChar(ch);
        }
    }
}

void UART_Printf(const char *fmt, ...)
{
    char buffer[280];

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len > 0)
    {
        if (len >= (int)sizeof(buffer))
        {
            len = sizeof(buffer) - 1;
        }

        for (int i = 0; i < len; i++) {
            UART_PutChar(buffer[i]);
        }
    }
}
