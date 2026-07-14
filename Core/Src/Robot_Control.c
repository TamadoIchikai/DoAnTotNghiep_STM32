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
#include "PoE_Kinematics.h"
#include "arm_math.h"
#include "Custom_CLI.h"

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

    sprintf(msg, "%s finished move.\r\n", motor->name);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

static uint8_t ik_active = 0;
static float32_t T_ik_target[16];

void Robot_Set_IK_Target(const float32_t T_d[16]) {
    memcpy(T_ik_target, T_d, sizeof(float32_t) * 16);
    ik_active = 1;
}

void Robot_IK_Stop(void) {
    ik_active = 0;
}

typedef struct {
    uint8_t active;
    uint8_t motor_id;
    float32_t start_angle;
    float32_t target_angle;
    uint8_t dir;
} JointMove_t;

static JointMove_t joint_moves[5] = {0};

void Robot_Set_Joint_Target(uint8_t motor_id, uint8_t dir, float32_t target_deg) {
    if (motor_id < 1 || motor_id > 4) return;
    joint_moves[motor_id].active = 1;
    joint_moves[motor_id].motor_id = motor_id;
    joint_moves[motor_id].dir = dir;

    float32_t start = 0.0f;
    if (motor_id == 1) start = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG);
    else if (motor_id == 2) start = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG);
    else if (motor_id == 3) start = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_DEG);
    else if (motor_id == 4) start = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG); 
    joint_moves[motor_id].start_angle = start;
    
    if (dir == 1) {
        joint_moves[motor_id].target_angle = start + target_deg;
    } else {
        joint_moves[motor_id].target_angle = start - target_deg;
    }

    if (motor_id == 1) Stepper_Start(&motor1, 1, dir);
    else if (motor_id == 2) Stepper_Start(&motor2, 2, dir);
    else if (motor_id == 4) Stepper_Start(&motor4, 3, dir);
}

void Robot_Joint_Task_Service(void) {
    if (e_stop_active) {
        for(int i = 1; i <= 4; i++) joint_moves[i].active = 0;
        return;
    }

    for (int i = 1; i <= 4; i++) {
        if (!joint_moves[i].active) continue;

        float32_t current_angle = 0.0f;
        if (i == 1) current_angle = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG);
        else if (i == 2) current_angle = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG);
        else if (i == 3) current_angle = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_DEG);
        else if (i == 4) current_angle = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG); 

        uint8_t reached = 0;
        if (joint_moves[i].start_angle <= joint_moves[i].target_angle) {
            // Target is above or equal to start, so wait until current crosses target upwards
            if (current_angle >= joint_moves[i].target_angle) reached = 1;
        } else {
            // Target is below start, so wait until current crosses target downwards
            if (current_angle <= joint_moves[i].target_angle) reached = 1;
        }

        if (reached) {
            joint_moves[i].active = 0;
            if (i == 1) Stepper_Stop(&motor1);
            else if (i == 2) Stepper_Stop(&motor2);
            else if (i == 4) Stepper_Stop(&motor4);
            UART_Printf("\r\n[!] Motor %d reached target %.2f deg\r\n> ", i, joint_moves[i].target_angle);
        }
    }
}

void Robot_IK_Task_20ms(void) {
    if (e_stop_active) {
        ik_active = 0;
        return;
    }
    if (!ik_active) return;
    
    // 1. Get current joint angles
    float32_t thetalist[4];
    thetalist[0] = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_RAD);
    thetalist[1] = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_RAD);
    thetalist[2] = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_RAD);
    float32_t m4_deg = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG);
    thetalist[3] = Angle_To_Distance(m4_deg, M4_Pulley_Linear_Dia, UNIT_DEG) / 1000.0f; // in meters

    // 2. Forward Kinematics
    float32_t T_EE[16];
    ForwardKinematics_Space(M_home, S, thetalist, 4, T_EE);

    // 3. Twist Error
    float32_t V_EE[6];
    Compute_Twist_Error(T_ik_target, T_EE, V_EE);

    float32_t err_norm = 0;
    for (int i = 0; i < 6; i++) err_norm += V_EE[i] * V_EE[i];
    
    // Stop condition
    if (err_norm < 1e-6f) {
        ik_active = 0;
        Stepper_Stop(&motor1);
        Stepper_Stop(&motor2);
        Stepper_Stop(&motor4);
        DC_Stop();
        UART_Printf("IK Reached Target.\r\n");
        return;
    }

    // 4. Jacobian
    float32_t Js[24]; // 6x4
    jacobianSpace(S, thetalist, 4, Js);

    // 5. Inverse Jacobian (DLS)
    float32_t Js_Inverse[24]; // 4x6
    DLS(Js, 4, Js_Inverse);

    // 6. Compute delta theta
    float32_t d_theta[4] = {0};
    arm_matrix_instance_f32 mat_Jinv, mat_V, mat_dtheta;
    arm_mat_init_f32(&mat_Jinv, 4, 6, Js_Inverse);
    arm_mat_init_f32(&mat_V, 6, 1, V_EE);
    arm_mat_init_f32(&mat_dtheta, 4, 1, d_theta);
    arm_mat_mult_f32(&mat_Jinv, &mat_V, &mat_dtheta);

    // 7. Apply to motors (Simple P control mapping velocity to stepper interval)
    // Motor 1
    if (fabsf(d_theta[0]) > 0.005f) {
        uint8_t dir = (d_theta[0] > 0) ? 1 : 0;
        uint32_t interval = (uint32_t)(50000.0f / fabsf(d_theta[0])); 
        if (interval < 200) interval = 200;
        if (interval > 5000) { Stepper_Stop(&motor1); }
        else {
            Stepper_Start(&motor1, 1, dir);
            motor1.targetIntervalUs = interval;
        }
    } else { Stepper_Stop(&motor1); }

    // Motor 2
    if (fabsf(d_theta[1]) > 0.005f) {
        uint8_t dir = (d_theta[1] > 0) ? 1 : 0;
        uint32_t interval = (uint32_t)(50000.0f / fabsf(d_theta[1])); 
        if (interval < 200) interval = 200;
        if (interval > 5000) { Stepper_Stop(&motor2); }
        else {
            Stepper_Start(&motor2, 2, dir);
            motor2.targetIntervalUs = interval;
        }
    } else { Stepper_Stop(&motor2); }

    // Motor 3 (DC Motor)
    if (fabsf(d_theta[2]) > 0.005f) {
        uint8_t dir = (d_theta[2] > 0) ? 1 : 0;
        if (!g_dc_running || g_dc_forward != dir) {
            DC_Start(dir);
        }
    } else { DC_Stop(); }

    // Motor 4 (Prismatic - d_theta[3] is in meters)
    if (fabsf(d_theta[3]) > 0.0005f) {
        uint8_t dir = (d_theta[3] > 0) ? 1 : 0; 
        uint32_t interval = (uint32_t)(1.0f / fabsf(d_theta[3])); 
        if (interval < 200) interval = 200;
        if (interval > 5000) { Stepper_Stop(&motor4); }
        else {
            Stepper_Start(&motor4, 4, dir);
            motor4.targetIntervalUs = interval;
        }
    } else { Stepper_Stop(&motor4); }
}

// Main parallel initialization routine using encoder feedback for offset tracking
void Robot_Init_State(float32_t init_Angle[4])
{
    UART_Printf("--- STARTING PARALLEL ROBOT INITIALIZATION ---\r\n");

    Stepper_t* m_ptrs[3] = {&motor1, &motor2, &motor4};
    uint8_t m_ids[3] = {1, 2, 4};

    // Pointers to the globally updated encoder variables
    volatile int32_t* enc_ptrs[3] = {&enc1_pos, &enc2_pos, &enc4_pos};

    // Original search directions: M1: CCW (1), M2: CW (0), M4: CCW (1)
    uint8_t search_dirs[3] = {1, 0, 1};

    float32_t angles[3] = {init_Angle[0], init_Angle[1], init_Angle[3]};

    uint8_t state[3] = {0, 0, 0}; // 0:Search, 1:Backoff, 2:Offset, 3:Done
    uint8_t target_dirs[3];
    int32_t zero_counts[3] = {0, 0, 0};
    uint32_t last_us[3];
    
    int32_t last_enc_val[3];
    uint32_t step_count[3] = {0, 0, 0};
    uint32_t triggered_time[3] = {0, 0, 0};
    uint8_t is_triggered[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++)
    {
        Stepper_SetDir(m_ptrs[i], search_dirs[i]);
        Stepper_Enable(m_ptrs[i]);

        // 1 maps to positive angle ('t' direction), 0 maps to negative angle ('n' direction)
        target_dirs[i] = (angles[i] >= 0.0f) ? 1 : 0;
        last_us[i] = micros();
        last_enc_val[i] = *enc_ptrs[i];
    }

    uint8_t all_done = 0;

    // Parallel state machine logic
    while (!all_done)
    {
        UART_Command_Service();
        if (e_stop_active) {
            UART_Printf("\r\n[!] INIT ABORTED BY E-STOP.\r\n");
            for(int i=0; i<3; i++) Stepper_Stop(m_ptrs[i]);
            break;
        }

        uint32_t now = micros();
        all_done = 1;
        
        // 2-second continuous activation timeout for limit switches
        for (int i = 0; i < 3; i++) {
            if (Motor_Limit_IsTriggered(m_ids[i])) {
                if (!is_triggered[i]) {
                    is_triggered[i] = 1;
                    triggered_time[i] = now;
                } else if ((now - triggered_time[i]) > 2000000) { // 2 seconds in micros
                    UART_Printf("\r\n[ERR] M%d LIMIT SWITCH ON > 2s! DISCONNECTED? ABORTING.\r\n", m_ids[i]);
                    e_stop_active = 1;
                }
            } else {
                is_triggered[i] = 0;
            }
        }

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
                            
                            step_count[i]++;
                            if (step_count[i] % 1000 == 0) {
                                int32_t diff = *enc_ptrs[i] - last_enc_val[i];
                                if (diff < 0) diff = -diff;
                                if (diff < 10) {
                                    UART_Printf("\r\n[ERR] M%d ENCODER DEAD! ABORTING.\r\n", m_ids[i]);
                                    e_stop_active = 1;
                                }
                                last_enc_val[i] = *enc_ptrs[i];
                            }
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
