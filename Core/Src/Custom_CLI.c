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
char rx_cmd[32];
uint8_t rx_index = 0;

extern void PoETest(void);

void Print_Status(void)
{
    UART_Printf("------------------------\r\n");

    UART_Printf("ENC1=%ld | ENC2=%ld | ENC3=%ld | ENC4=%ld\r\n",
                (long)enc1_pos,
                (long)enc2_pos,
                (long)enc3_pos,
                (long)enc4_pos);

    // Calculate angles in degrees
	float ang1 = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG);
	float ang2 = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG);
	float ang3 = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_DEG);
	float ang4 = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG);

	// Print calculated angles
	UART_Printf("ANG1=%.2f | ANG2=%.2f | ANG3=%.2f | ANG4=%.2f\r\n",
				ang1,
				ang2,
				ang3,
				ang4);

    UART_Printf("LIM=%d%d%d%d%d%d\r\n",
                Limit1_IsTriggered(),
                Limit2_IsTriggered(),
                Limit3_IsTriggered(),
                Limit4_IsTriggered(),
                Limit5_IsTriggered(),
                Limit6_IsTriggered());

    UART_Printf("M=%d%d%d%d | MAG=%d | US(1,2,4)=%lu,%lu,%lu | DC_PWM=%u\r\n",
                    motor1.running,
                    motor2.running,
                    g_dc_running,
                    motor4.running,
                    g_magnet_on,
                    (unsigned long)motor1.targetIntervalUs,
                    (unsigned long)motor2.targetIntervalUs,
                    (unsigned long)motor4.targetIntervalUs,
                    g_dc_pwm);


    UART_Printf("------------------------\r\n");
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

    if (cmd[0] == 'x')
    {
        Stepper_All_Stop();
        DC_Stop();
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

void UART_Command_Service(void)
{
    uint8_t ch;

    while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
    {
        if (ch == '\r' || ch == '\n' || ch == ' ')
        {
            if (rx_index > 0)
            {
                rx_cmd[rx_index] = '\0';
                UART_Printf("CMD: %s\r\n", rx_cmd);
                Process_Command(rx_cmd);
                rx_index = 0;
            }
        }
        else
        {
            if (rx_index < sizeof(rx_cmd) - 1)
            {
                rx_cmd[rx_index++] = (char)ch;
            }
            else
            {
                rx_index = 0;
            }

            // Cho phép lệnh 1 ký tự chạy ngay: x, z
            if (rx_index == 1)
            {
                if (rx_cmd[0] == 'x' || rx_cmd[0] == 'X' ||
                    rx_cmd[0] == 'z' || rx_cmd[0] == 'Z')
                {
                    rx_cmd[rx_index] = '\0';
                    UART_Printf("CMD: %s\r\n", rx_cmd);
                    Process_Command(rx_cmd);
                    rx_index = 0;
                }
            }

            // Cho phép lệnh 2 ký tự chạy ngay: t1, n1, s1, m1...
            if (rx_index == 2)
            {
                char c0 = (char)tolower((unsigned char)rx_cmd[0]);
                char c1 = rx_cmd[1];

                if ((c0 == 't' || c0 == 'n' || c0 == 's' || c0 == 'm'|| c0 == 'c') &&
                    (c1 >= '0' && c1 <= '9'))
                {
                    rx_cmd[rx_index] = '\0';
                    UART_Printf("CMD: %s\r\n", rx_cmd);
                    Process_Command(rx_cmd);
                    rx_index = 0;
                }
            }
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

        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
    }
}
