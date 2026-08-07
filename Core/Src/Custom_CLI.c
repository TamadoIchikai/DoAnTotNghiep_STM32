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
#include "PoE_Kinematics.h"
#include "Custom_CLI.h"
#include "Custom_utils.h"
#include "HW_Motors.h"
#include "HW_Encoders.h"
#include "HW_LimitSwitches.h"
#include "Robot_Control.h"
#include "Controllers.h"

uint8_t e_stop_active = 0;
char rx_cmd[64];
uint8_t rx_index = 0;

extern Stepper_t motor1;
extern Stepper_t motor2;
extern Stepper_t motor4;

extern uint8_t g_dc_running;
extern uint8_t g_dc_forward;

// --- TX Ring Buffer ---
#define TX_BUF_SIZE 2048
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

// Redirect standard printf to the UART ring buffer
int __io_putchar(int ch) {
    UART_PutChar((char)ch);
    return ch;
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

    static int32_t prev_enc1 = 0, prev_enc2 = 0, prev_enc3 = 0, prev_enc4 = 0;
    static uint32_t prev_time = 0;
    uint32_t current_time = HAL_GetTick();
    float delta_t = (current_time - prev_time) / 1000.0f;
    if (delta_t <= 0.0f) delta_t = 0.001f;

    float vel1 = Encoder_To_Velocity(enc1_pos, prev_enc1, delta_t, ENCODER_PPR, UNIT_DEG);
    float vel2 = Encoder_To_Velocity(enc2_pos, prev_enc2, delta_t, ENCODER_PPR, UNIT_DEG);
    float vel3 = Encoder_To_Velocity(enc3_pos, prev_enc3, delta_t, 11, UNIT_DEG);
    float vel4_deg = Encoder_To_Velocity(enc4_pos, prev_enc4, delta_t, ENCODER_PPR, UNIT_DEG);
    float vel4_mm_s = Angle_To_Distance(vel4_deg, M4_Pulley_Linear_Dia, UNIT_DEG);

    prev_enc1 = enc1_pos;
    prev_enc2 = enc2_pos;
    prev_enc3 = enc3_pos;
    prev_enc4 = enc4_pos;
    prev_time = current_time;

    UART_PutString("ANG1="); UART_PutFloatFast(ang1);
    UART_PutString(" | ANG2="); UART_PutFloatFast(ang2);
    UART_PutString(" | ANG3="); UART_PutFloatFast(ang3);
    UART_PutString(" | DIST4="); UART_PutFloatFast(dist4); UART_PutString("mm\r\n");

    UART_PutString("VEL1="); UART_PutFloatFast(vel1);
    UART_PutString(" | VEL2="); UART_PutFloatFast(vel2);
    UART_PutString(" | VEL3="); UART_PutFloatFast(vel3);
    UART_PutString(" | VEL4="); UART_PutFloatFast(vel4_mm_s); UART_PutString("mm/s\r\n");

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
        extern float32_t robot_Init_Pose[4];
        extern float32_t M_home_matlab[16];

        float32_t thetalist[4];
        thetalist[0] = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[0];
        thetalist[1] = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[1];
        thetalist[2] = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[2];
        float32_t m4_rad = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_RAD);
        thetalist[3] = (Angle_To_Distance(m4_rad, M4_Pulley_Linear_Dia, UNIT_RAD) / 1000.0f) + robot_Init_Pose[3];

        float32_t T_EE[16];
        ForwardKinematics_Space(M_home_matlab, S, thetalist, 4, T_EE);

        UART_Printf("Forward Kinematics T_EE (4x4):\r\n");
        for (int i = 0; i < 4; i++) {
            UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n", 
                T_EE[i*4], T_EE[i*4+1], T_EE[i*4+2], T_EE[i*4+3]);
        }
        return;
    }

    if (strncmp(cmd, "transfer_m", 10) == 0)
    {
        int motor_id = 0;
        uint32_t freq = 0;
        char dir_char = 0;
        float target_deg = 0.0f;
        
        if (sscanf(cmd, "transfer_m%d_%lu_%c%f", &motor_id, &freq, &dir_char, &target_deg) == 4) {
            Transfer_Test_Start((uint8_t)motor_id, freq, dir_char, target_deg);
        } else {
            UART_Printf("Invalid transfer command. Format: transfer_m[id]_[freq]_[t/n][deg]\r\n");
        }
        return;
    }

    if (strcmp(cmd, "ik_run") == 0)
    {
        UART_Printf("IK Target T_d (4x4):\r\n");
        for (int i = 0; i < 4; i++) {
            UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n",
                T_d[i*4], T_d[i*4+1], T_d[i*4+2], T_d[i*4+3]);
        }
        Robot_Set_IK_Target(T_d);
        UART_Printf("IK active. Moving to target (Open-loop)...\r\n");
        return;
    }
    
    if (strcmp(cmd, "ik_run_con_1") == 0)
    {
        UART_Printf("IK Target T_d (4x4):\r\n");
        for (int i = 0; i < 4; i++) {
            UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n",
                T_d[i*4], T_d[i*4+1], T_d[i*4+2], T_d[i*4+3]);
        }
        Robot_Set_IK_Target_PD(T_d, 1);
        UART_Printf("IK active. Moving using Multi-PD param set 1...\r\n");
        return;
    }

    if (strcmp(cmd, "ik_run_con_2") == 0)
    {
        UART_Printf("IK Target T_d (4x4):\r\n");
        for (int i = 0; i < 4; i++) {
            UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n",
                T_d[i*4], T_d[i*4+1], T_d[i*4+2], T_d[i*4+3]);
        }
        Robot_Set_IK_Target_PD(T_d, 2);
        UART_Printf("IK active. Moving using Multi-PD param set 2...\r\n");
        return;
    }

    if (strcmp(cmd, "ik_mulrun") == 0)
    {
        UART_Printf("Starting Multi-IK Sequence (T_d -> T_d4) [OPEN LOOP]...\r\n");
        Robot_Start_Multi_IK();
        return;
    }

    if (strcmp(cmd, "ik_mulrun_con_1") == 0)
    {
        UART_Printf("Starting Multi-IK Sequence (T_d -> T_d4) [CLOSED LOOP CON 1]...\r\n");
        Robot_Start_Multi_IK_PD();
        return;
    }

    if (strncmp(cmd, "ptp_run_con_", 12) == 0)
    {
        int con_id = 0;
        float thetaD = 0.0f;
        float pos_x = 0.0f;
        float pos_y = 0.0f;
        float pos_z = 0.0f;

        if (sscanf(cmd, "ptp_run_con_%d_%f_%f_%f_%f", &con_id, &thetaD, &pos_x, &pos_y, &pos_z) == 5) {
            if (con_id != 1 && con_id != 2) {
                UART_Printf("Invalid controller ID. Use 1 or 2.\r\n");
                return;
            }
            
            extern float32_t T_d[16];
            Convert_PTP_to_Td(thetaD, pos_x, pos_y, pos_z, T_d);
            
            UART_Printf("IK Target T_d (4x4) from ptp:\r\n");
            for (int i = 0; i < 4; i++) {
                UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n",
                    T_d[i*4], T_d[i*4+1], T_d[i*4+2], T_d[i*4+3]);
            }
            
            extern uint8_t PTP_RUN_CON_FLAG;
            extern float32_t ptp_pos_D_x, ptp_pos_D_y, ptp_pos_D_z, ptp_angle_D;
            
            PTP_RUN_CON_FLAG = 1;
            ptp_pos_D_x = pos_x;
            ptp_pos_D_y = pos_y;
            ptp_pos_D_z = pos_z;
            ptp_angle_D = atan2f(T_d[4], T_d[0]) * 180.0f / PI;

            Robot_Set_IK_Target_PD(T_d, con_id);
            UART_Printf("IK active. Moving using Multi-PD param set %d...\r\n", con_id);
        } else {
            UART_Printf("Format: ptp_run_con_<id>_<thetaD>_<X>_<Y>_<Z>\r\n");
        }
        return;
    }

    if (strncmp(cmd, "ptp_run_", 8) == 0 && strncmp(cmd, "ptp_run_con_", 12) != 0)
    {
        float thetaD = 0.0f;
        float pos_x = 0.0f;
        float pos_y = 0.0f;
        float pos_z = 0.0f;

        if (sscanf(cmd, "ptp_run_%f_%f_%f_%f", &thetaD, &pos_x, &pos_y, &pos_z) == 4) {
            extern float32_t T_d[16];
            Convert_PTP_to_Td(thetaD, pos_x, pos_y, pos_z, T_d);
            
            UART_Printf("IK Target T_d (4x4) from ptp:\r\n");
            for (int i = 0; i < 4; i++) {
                UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n",
                    T_d[i*4], T_d[i*4+1], T_d[i*4+2], T_d[i*4+3]);
            }
            
            extern uint8_t PTP_RUN_OPEN_FLAG;
            extern uint32_t ptp_open_start_time;
            extern float32_t ptp_pos_D_x, ptp_pos_D_y, ptp_pos_D_z, ptp_angle_D;
            
            PTP_RUN_OPEN_FLAG = 1;
            ptp_open_start_time = HAL_GetTick();
            ptp_pos_D_x = pos_x;
            ptp_pos_D_y = pos_y;
            ptp_pos_D_z = pos_z;
            ptp_angle_D = atan2f(T_d[4], T_d[0]) * 180.0f / PI;

            UART_Printf("\r\n[MPD_START] Open-Loop PTP Started\r\n");
            Robot_Set_IK_Target(T_d);
            UART_Printf("IK active. Moving to target (Open-loop)...\r\n");
        } else {
            UART_Printf("Format: ptp_run_<thetaD>_<X>_<Y>_<Z>\r\n");
        }
        return;
    }

    if (strncmp(cmd, "auto_con_1_mag_on_", 18) == 0) {
        float angle = 0.0f;
        float pick_x = 0.0f, pick_y = 0.0f;
        float place_x = 0.0f, place_y = 0.0f;
        
        // Check if this is a pick-and-place command (contains "_mag_off_")
        const char *mag_off_ptr = strstr(cmd + 18, "_mag_off_");
        
        if (mag_off_ptr != NULL) {
            // Full pick-and-place: auto_con_1_mag_on_<angle>_<x>_<y>_mag_off_<x>_<y>[_mm]
            if (sscanf(cmd, "auto_con_1_mag_on_%f_%f_%f", &angle, &pick_x, &pick_y) != 3) {
                UART_Printf("Format: auto_con_1_mag_on_<angle>_<pickX>_<pickY>_mag_off_<placeX>_<placeY>[_mm]\r\n");
                return;
            }
            if (sscanf(mag_off_ptr, "_mag_off_%f_%f", &place_x, &place_y) != 2) {
                UART_Printf("Format: auto_con_1_mag_on_<angle>_<pickX>_<pickY>_mag_off_<placeX>_<placeY>[_mm]\r\n");
                return;
            }
            
            // Check for _mm suffix -> convert mm to meters
            size_t cmd_len = strlen(cmd);
            if (cmd_len >= 3 && strcmp(cmd + cmd_len - 3, "_mm") == 0) {
                pick_x  /= 1000.0f;
                pick_y  /= 1000.0f;
                place_x /= 1000.0f;
                place_y /= 1000.0f;
                UART_Printf("[AUTO] Unit: mm -> converted to meters\r\n");
            }
            
            Robot_Start_Auto_Pick_Place(angle, pick_x, pick_y, place_x, place_y);
        } else {
            // Pick-only: auto_con_1_mag_on_<angle>_<x>_<y>
            if (sscanf(cmd, "auto_con_1_mag_on_%f_%f_%f", &angle, &pick_x, &pick_y) == 3) {
                Robot_Start_Auto_Sequence(angle, pick_x, pick_y);
            } else {
                UART_Printf("Format: auto_con_1_mag_on_<angle>_<X>_<Y>\r\n");
            }
        }
        return;
    }

    if (strncmp(cmd, "pid_run_m", 9) == 0 && strlen(cmd) >= 13)
    {
        uint8_t motor_id = cmd[9] - '0';
        char dir_char = cmd[11]; // e.g. 't' or 'n'
        uint8_t param_idx = cmd[12] - '0';
        
        if (motor_id < 1 || motor_id > 4 || (dir_char != 't' && dir_char != 'n') || (param_idx < 1 || param_idx > 2)) {
            UART_Printf("Invalid pid_run syntax. Use pid_run_m[1-4]_[t/n][1/2]\r\n");
            return;
        }

        extern float32_t kpd_M1_1[2], kpd_M1_2[2];
        extern float32_t kpd_M2_1[2], kpd_M2_2[2];
        extern float32_t kpd_M3_1[2], kpd_M3_2[2];
        extern float32_t kpd_M4_1[2], kpd_M4_2[2];
        
        float Kp = 0.0f, Kd = 0.0f;
        float32_t *kpd = NULL;

        if (motor_id == 1) kpd = (param_idx == 1) ? kpd_M1_1 : kpd_M1_2;
        else if (motor_id == 2) kpd = (param_idx == 1) ? kpd_M2_1 : kpd_M2_2;
        else if (motor_id == 3) kpd = (param_idx == 1) ? kpd_M3_1 : kpd_M3_2;
        else if (motor_id == 4) kpd = (param_idx == 1) ? kpd_M4_1 : kpd_M4_2;

        if (kpd) {
            Kp = kpd[0];
            Kd = kpd[1];
        }

        uint8_t forward = (dir_char == 't') ? 1 : 0;
        
        int32_t enc_pos_val = 0;
        if (motor_id == 1) enc_pos_val = enc1_pos;
        else if (motor_id == 2) enc_pos_val = enc2_pos;
        else if (motor_id == 3) enc_pos_val = enc3_pos;
        else if (motor_id == 4) enc_pos_val = enc4_pos;

        float start_deg = Encoder_To_Angle(enc_pos_val, ENCODER_PPR, UNIT_DEG);
        float target_deg = start_deg + (forward ? 90.0f : -90.0f);
        
        UART_Printf("Testing M%d PD to %.2f deg with Kp=%.2f Kd=%.2f\r\n", motor_id, target_deg, Kp, Kd);
        
        PD_Test_Start(motor_id, target_deg, Kp, Kd);
        return;
    }

    if (strncmp(cmd, "pid_m", 5) == 0)
    {
        // Format: pid_m<id>_<dir><target>_<kp>_[kd]_[time_s] e.g., pid_m1_n180_25.5_0.3_10
        int m_id = 0;
        char dir_c = 0;
        float target = 0.0f;
        float kp = 0.0f;
        float kd = 0.0f; // Default to 0 if not provided
        float time_s = 10.0f; // Default to 10s if not provided
        
        char *ptr = cmd + 5; // Skip "pid_m"
        m_id = atoi(ptr);
        
        ptr = strchr(ptr, '_');
        if (ptr) {
            ptr++;
            dir_c = *ptr;
            target = atof(ptr + 1);
            
            ptr = strchr(ptr, '_');
            if (ptr) {
                kp = atof(ptr + 1);
                
                ptr = strchr(ptr + 1, '_');
                if (ptr) {
                    kd = atof(ptr + 1);
                    
                    ptr = strchr(ptr + 1, '_');
                    if (ptr) {
                        time_s = atof(ptr + 1);
                    }
                }
                Matlab_PID_Test_Start((uint8_t)m_id, dir_c, target, kp, kd, time_s);
                return;
            }
        }
        UART_Printf("Invalid format. Use pid_m<id>_<t/n><target>_<kp>_[kd]_[time_s]\r\n");
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
    char buffer[512];

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
