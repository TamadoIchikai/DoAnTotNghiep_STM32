/*
 * Robot_Control.c
 */

#include "Robot_Control.h"
#include "Custom_CLI.h"       // For UART_Printf
#include "HW_LimitSwitches.h" // For Motor_Limit_IsTriggered
#include "HW_Encoders.h"      // For Encoders_Reset, Encoder_To_Angle
#include "Custom_utils.h"

uint8_t PTP_RUN_OPEN_FLAG = 0;
uint32_t ptp_open_start_time = 0;     // For delay_us, micros
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "PoE_Kinematics.h"
#include "arm_math.h"
#include "robot_uart_debug.h"
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

void Robot_Set_IK_Target(const float32_t T_d[16]) {
    UART_Printf("Solving Inverse Kinematics...\r\n");
    
    // Safeguard 1: Input Bounds (Reachability)
    float32_t req_r = sqrtf(T_d[3]*T_d[3] + T_d[7]*T_d[7]);
    if (req_r > 0.3501f) {
        UART_Printf("[!] IK Input out of reach (XY: %.2f > 0.35m)\r\n", req_r);
        return;
    }


    extern float32_t robot_Init_Pose[4];
    extern float32_t M_home_matlab[16];
    extern float32_t M1_Lim_Deg, M2_Lim_Deg, M3_Lim_Deg, M4_Lim_mm;

    float32_t T_EE[16];
    float32_t V_EE[6];
    float32_t Js[24];
    float32_t Js_Inverse[24];
    float32_t d_theta[4];
    
    arm_matrix_instance_f32 mat_Jinv, mat_V, mat_dtheta;
    arm_mat_init_f32(&mat_Jinv, 4, 6, Js_Inverse);
    arm_mat_init_f32(&mat_V, 6, 1, V_EE);
    arm_mat_init_f32(&mat_dtheta, 4, 1, d_theta);

    uint8_t solution_found = 0;
    float32_t target_abs_deg[4] = {0};

    // Try up to 2 attempts (Attempt 0: current elbow, Attempt 1: flipped elbow)
    for (int attempt = 0; attempt < 2; attempt++) {
        // 1. Get current absolute joint angles as initial guess
        float32_t thetalist[4];
        thetalist[0] = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[0];
        thetalist[1] = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[1];
        thetalist[2] = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[2];
        float32_t m4_rad = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_RAD);
        thetalist[3] = (Angle_To_Distance(m4_rad, M4_Pulley_Linear_Dia, UNIT_RAD) / 1000.0f) + robot_Init_Pose[3];

        if (attempt == 1) {
            // Flip the elbow guess for the second attempt to find the alternate IK solution
            thetalist[1] = -thetalist[1];
        }

        uint8_t converged = 0;
        // Max 100 iterations for Newton-Raphson
        for (int iter = 0; iter < 100; iter++) {
            ForwardKinematics_Space(M_home_matlab, S, thetalist, 4, T_EE);
            Compute_Twist_Error(T_d, T_EE, V_EE);

            float32_t err_norm = 0;
            for (int i = 0; i < 6; i++) err_norm += V_EE[i] * V_EE[i];
            
            if (err_norm < 1e-6f) {
                converged = 1;
                break;
            }

            jacobianSpace(S, thetalist, 4, Js);
            DLS(Js, 4, Js_Inverse);
            arm_mat_mult_f32(&mat_Jinv, &mat_V, &mat_dtheta);

            for (int i = 0; i < 4; i++) {
                thetalist[i] += d_theta[i];
            }
        }

        if (converged) {
            // Convert thetalist (absolute state) back to physical angles from physical zero (in degrees)
            float32_t deg1 = (thetalist[0] - robot_Init_Pose[0]) * 180.0f / PI;
            float32_t deg2 = (thetalist[1] - robot_Init_Pose[1]) * 180.0f / PI;
            float32_t deg3 = (thetalist[2] - robot_Init_Pose[2]) * 180.0f / PI;
            
            // For M4, thetalist[3] is absolute translation in meters
            float32_t dist_m = thetalist[3] - robot_Init_Pose[3];
            float32_t dist_mm = dist_m * 1000.0f;

            // Safeguard 2: Output Joint Limits
            if (fabsf(deg1) <= M1_Lim_Deg &&
                fabsf(deg2) <= M2_Lim_Deg &&
                fabsf(deg3) <= M3_Lim_Deg &&
                fabsf(dist_mm) <= M4_Lim_mm) 
            {
                // Solution is valid and within physical limits
                target_abs_deg[0] = deg1;
                target_abs_deg[1] = deg2;
                target_abs_deg[2] = deg3;
                target_abs_deg[3] = (dist_mm / (M4_Pulley_Linear_Dia / 2.0f)) * 180.0f / PI;
                solution_found = 1;
                UART_Printf("IK Converged! Dispatching joints.\r\n");
                break; 
            } else {
                UART_Printf("[!] IK Solution %d exceeds joint limits.\r\n", attempt);
            }
        }
    }

    if (!solution_found) {
        UART_Printf("[!] IK Solver failed to find a valid reachable solution.\r\n");
        return;
    }

    // Dispatch to Joint Move Service
    for (int i = 1; i <= 4; i++) {
        float32_t current_angle = 0;
        if (i == 1) current_angle = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG);
        else if (i == 2) current_angle = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG);
        else if (i == 3) current_angle = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_DEG);
        else if (i == 4) current_angle = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG);

        float32_t relative_disp = target_abs_deg[i-1] - current_angle;
        if (fabsf(relative_disp) > 0.05f) { // Only move if significant
            uint8_t dir = (relative_disp > 0) ? 1 : 0;
            Robot_Set_Joint_Target(i, dir, fabsf(relative_disp));
        }
    }
}

void Robot_Set_IK_Target_PD(const float32_t T_d[16], uint8_t param_idx) {
    UART_Printf("Solving Inverse Kinematics for Multi-PD...\r\n");
    
    // Safeguard 1: Input Bounds (Reachability)
    float32_t req_r = sqrtf(T_d[3]*T_d[3] + T_d[7]*T_d[7]);
    if (req_r > 0.3501f) {
        UART_Printf("[!] IK Input out of reach (XY: %.2f > 0.35m)\r\n", req_r);
        return;
    }

    extern float32_t robot_Init_Pose[4];
    extern float32_t M_home_matlab[16];
    extern float32_t M1_Lim_Deg, M2_Lim_Deg, M3_Lim_Deg, M4_Lim_mm;
    extern float32_t S[24];

    float32_t T_EE[16];
    float32_t V_EE[6];
    float32_t Js[24];
    float32_t Js_Inverse[24];
    float32_t d_theta[4];
    
    arm_matrix_instance_f32 mat_Jinv, mat_V, mat_dtheta;
    arm_mat_init_f32(&mat_Jinv, 4, 6, Js_Inverse);
    arm_mat_init_f32(&mat_V, 6, 1, V_EE);
    arm_mat_init_f32(&mat_dtheta, 4, 1, d_theta);

    uint8_t solution_found = 0;
    float32_t target_abs_deg[4] = {0};

    for (int attempt = 0; attempt < 2; attempt++) {
        float32_t thetalist[4];
        thetalist[0] = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[0];
        thetalist[1] = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[1];
        thetalist[2] = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[2];
        float32_t m4_rad = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_RAD);
        extern float32_t M4_Pulley_Linear_Dia;
        thetalist[3] = (Angle_To_Distance(m4_rad, M4_Pulley_Linear_Dia, UNIT_RAD) / 1000.0f) + robot_Init_Pose[3];

        if (attempt == 1) {
            thetalist[1] = -thetalist[1]; // Try alternate elbow solution
        }

        uint8_t converged = 0;
        for (int iter = 0; iter < 100; iter++) {
            ForwardKinematics_Space(M_home_matlab, S, thetalist, 4, T_EE);
            Compute_Twist_Error(T_d, T_EE, V_EE);

            float32_t err_norm = 0;
            for (int i = 0; i < 6; i++) err_norm += V_EE[i] * V_EE[i];
            
            if (err_norm < 1e-6f) {
                converged = 1;
                break;
            }

            jacobianSpace(S, thetalist, 4, Js);
            DLS(Js, 4, Js_Inverse);
            arm_mat_mult_f32(&mat_Jinv, &mat_V, &mat_dtheta);

            for (int i = 0; i < 4; i++) {
                thetalist[i] += d_theta[i];
            }
        }

        if (converged) {
            float32_t deg1 = (thetalist[0] - robot_Init_Pose[0]) * 180.0f / PI;
            float32_t deg2 = (thetalist[1] - robot_Init_Pose[1]) * 180.0f / PI;
            float32_t deg3 = (thetalist[2] - robot_Init_Pose[2]) * 180.0f / PI;
            float32_t dist_m = thetalist[3] - robot_Init_Pose[3];
            float32_t dist_mm = dist_m * 1000.0f;

            if (fabsf(deg1) <= M1_Lim_Deg && fabsf(deg2) <= M2_Lim_Deg &&
                fabsf(deg3) <= M3_Lim_Deg && fabsf(dist_mm) <= M4_Lim_mm) 
            {
                target_abs_deg[0] = deg1;
                target_abs_deg[1] = deg2;
                target_abs_deg[2] = deg3;
                target_abs_deg[3] = (dist_mm / (M4_Pulley_Linear_Dia / 2.0f)) * 180.0f / PI;
                solution_found = 1;
                UART_Printf("IK Converged! Dispatching to Multi-PD Controller.\r\n");
                break; 
            }
        }
    }

    if (!solution_found) {
        UART_Printf("[!] IK Solver failed to find a valid reachable solution.\r\n");
        return;
    }

    // Call the newly created Multi_PD_Start function
    extern void Multi_PD_Start(float32_t target_abs_deg[4], uint8_t param_idx);
    Multi_PD_Start(target_abs_deg, param_idx);
}




typedef struct {
    uint8_t active;
    uint8_t motor_id;
    float32_t start_angle;
    float32_t target_angle;
    uint8_t dir;
} JointMove_t;

static JointMove_t joint_moves[5] = {0};

void Robot_IK_Stop(void) {
    for(int i = 1; i <= 4; i++) joint_moves[i].active = 0;
    Stepper_Stop(&motor1);
    Stepper_Stop(&motor2);
    Stepper_Stop(&motor4);
    DC_Stop();
}

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
    else if (motor_id == 3) DC_Start(dir);
    else if (motor_id == 4) Stepper_Start(&motor4, 4, dir);
}

void Robot_Joint_Task_Service(void) {
    if (e_stop_active) {
        for(int i = 1; i <= 4; i++) joint_moves[i].active = 0;
        PTP_RUN_OPEN_FLAG = 0;
        return;
    }

    uint8_t any_active = 0;

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
            else if (i == 3) DC_Stop();
            else if (i == 4) Stepper_Stop(&motor4);
            UART_Printf("\r\n[!] Motor %d reached target %.2f deg\r\n> ", i, joint_moves[i].target_angle);
        }
        
        if (joint_moves[i].active) any_active = 1;
    }

    if (PTP_RUN_OPEN_FLAG) {
        static uint32_t last_log_time = 0;
        if (any_active && HAL_GetTick() - last_log_time >= 10) {
            last_log_time = HAL_GetTick();
            
            extern float32_t robot_Init_Pose[4];
            extern float32_t M_home_matlab[16];
            extern float32_t S[24];
            extern float32_t M4_Pulley_Linear_Dia;

            float32_t thetalist[4];
            thetalist[0] = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[0];
            thetalist[1] = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[1];
            thetalist[2] = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_RAD) + robot_Init_Pose[2];
            float32_t m4_rad = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_RAD);
            thetalist[3] = (Angle_To_Distance(m4_rad, M4_Pulley_Linear_Dia, UNIT_RAD) / 1000.0f) + robot_Init_Pose[3];

            float32_t T_EE[16];
            ForwardKinematics_Space(M_home_matlab, S, thetalist, 4, T_EE);

            float32_t x = T_EE[3];
            float32_t y = T_EE[7];
            float32_t z = T_EE[11];
            float32_t angle_Out = atan2f(T_EE[4], T_EE[0]) * 180.0f / PI;

            extern float32_t ptp_pos_D_x, ptp_pos_D_y, ptp_pos_D_z, ptp_angle_D;
            
            float32_t oa[4];
            oa[0] = Encoder_To_Angle(enc1_pos, ENCODER_PPR, UNIT_DEG);
            oa[1] = Encoder_To_Angle(enc2_pos, ENCODER_PPR, UNIT_DEG);
            oa[2] = Encoder_To_Angle(enc3_pos, ENCODER_PPR, UNIT_DEG);
            oa[3] = Encoder_To_Angle(enc4_pos, ENCODER_PPR, UNIT_DEG);
            float32_t ref4_disp_mm = Angle_To_Distance(joint_moves[4].target_angle, M4_Pulley_Linear_Dia, UNIT_DEG);
            float32_t enc4_disp_mm = Angle_To_Distance(oa[3], M4_Pulley_Linear_Dia, UNIT_DEG);
            
            UART_Printf("[PTP_DATA] %lu, %.4f,%.4f,%.4f,%.4f, %.4f,%.4f,%.4f,%.4f, 0,0,0,0, 0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0, %.4f,%.4f,%.4f,%.4f,%.4f, %.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                        HAL_GetTick() - ptp_open_start_time, ptp_pos_D_x, ptp_pos_D_y, ptp_pos_D_z, ptp_angle_D,
                        x, y, z, angle_Out,
                        joint_moves[1].target_angle, joint_moves[2].target_angle, joint_moves[3].target_angle, joint_moves[4].target_angle, ref4_disp_mm,
                        oa[0], oa[1], oa[2], oa[3], enc4_disp_mm);
        }
        
        if (!any_active) {
            PTP_RUN_OPEN_FLAG = 0;
            UART_Printf("[MPD_END] REACHED\r\n> ");
        }
    }
}

// MULTI-IK Logic
static uint8_t multi_ik_active = 0;
static uint8_t multi_ik_step = 0;
static const float32_t* multi_ik_targets[4];

uint8_t Robot_Is_Moving(void) {
    extern uint8_t multi_pd_active;
    if (multi_pd_active) return 1;

    for (int i=1; i<=4; i++) {
        if (joint_moves[i].active) return 1;
    }
    return 0;
}

void Robot_Multi_IK_Service(void) {
    if (!multi_ik_active) return;
    if (e_stop_active) { multi_ik_active = 0; return; }

    if (!Robot_Is_Moving()) {
        // Current move is done, dispatch next
        if (multi_ik_step >= 4) {
            multi_ik_active = 0;
            UART_Printf("[Multi-IK] Sequence complete!\r\n> ");
            return;
        }
        
        UART_Printf("\r\n[Multi-IK] Moving to point %d/4...\r\n", multi_ik_step + 1);
        Robot_Set_IK_Target(multi_ik_targets[multi_ik_step]);
        multi_ik_step++;
    }
}

void Robot_Start_Multi_IK(void) {
    extern float32_t T_d[16], T_d2[16], T_d3[16], T_d4[16];
    multi_ik_targets[0] = T_d;
    multi_ik_targets[1] = T_d2;
    multi_ik_targets[2] = T_d3;
    multi_ik_targets[3] = T_d4;
    multi_ik_step = 0;
    multi_ik_active = 1;
}

static uint8_t multi_ik_pd_active = 0;
static uint8_t multi_ik_pd_step = 0;
static const float32_t* multi_ik_pd_targets[4];

void Robot_Multi_IK_PD_Service(void) {
    if (!multi_ik_pd_active) return;
    if (e_stop_active) { multi_ik_pd_active = 0; return; }

    extern uint8_t multi_pd_active;
    if (!multi_pd_active) {
        // Current move is done, dispatch next
        if (multi_ik_pd_step >= 4) {
            multi_ik_pd_active = 0;
            UART_Printf("[Multi-IK-PD] Sequence complete!\r\n> ");
            return;
        }
        
        UART_Printf("\r\n[Multi-IK-PD] Moving to point %d/4...\r\n", multi_ik_pd_step + 1);
        Robot_Set_IK_Target_PD(multi_ik_pd_targets[multi_ik_pd_step], 1);
        multi_ik_pd_step++;
    }
}

void Robot_Start_Multi_IK_PD(void) {
    extern float32_t T_d[16], T_d2[16], T_d3[16], T_d4[16];
    multi_ik_pd_targets[0] = T_d;
    multi_ik_pd_targets[1] = T_d2;
    multi_ik_pd_targets[2] = T_d3;
    multi_ik_pd_targets[3] = T_d4;
    multi_ik_pd_step = 0;
    multi_ik_pd_active = 1;
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

// ==============================================================
// AUTOMATED PICK-AND-PLACE SEQUENCE
// ==============================================================
#include "HW_EndEffector.h" // For Magnet_On/Off

static Auto_Sequence_State_t auto_state = AUTO_STATE_IDLE;
static float auto_angle = 0.0f;
static float auto_pick_x = 0.0f;
static float auto_pick_y = 0.0f;
static float auto_place_x = 0.0f;
static float auto_place_y = 0.0f;
static uint8_t auto_has_place = 0; // 0 = pick-only (old command), 1 = full pick-and-place
static uint32_t auto_delay_start = 0;

// Original pick-only sequence (backwards compatible)
void Robot_Start_Auto_Sequence(float angle, float x, float y) {
    if (auto_state != AUTO_STATE_IDLE) {
        UART_Printf("Error: Auto sequence already running.\r\n");
        return;
    }
    auto_angle = angle;
    auto_pick_x = x;
    auto_pick_y = y;
    auto_has_place = 0;
    auto_state = AUTO_STATE_MOVE_XY_PICK;
    UART_Printf("\r\n[AUTO] Starting automated pick sequence...\r\n");
    
    float32_t T_d[16];
    Convert_PTP_to_Td(auto_angle, auto_pick_x, auto_pick_y, 0.08f, T_d);
    Robot_Set_IK_Target_PD(T_d, 1);
}

// Full pick-and-place sequence
void Robot_Start_Auto_Pick_Place(float angle, float pick_x, float pick_y, float place_x, float place_y) {
    if (auto_state != AUTO_STATE_IDLE) {
        UART_Printf("Error: Auto sequence already running.\r\n");
        return;
    }
    
    UART_Printf("\r\n[AUTO] Starting automated pick-and-place sequence...\r\n");
    UART_Printf("[AUTO] Pick: (%.4f, %.4f)  Place: (%.4f, %.4f)  Angle: %.1f deg\r\n",
                pick_x, pick_y, place_x, place_y, angle);

    // Pre-check: verify both coordinates are within reachable workspace
    float pick_r  = sqrtf(pick_x * pick_x + pick_y * pick_y);
    float place_r = sqrtf(place_x * place_x + place_y * place_y);
    if (pick_r > 0.3501f || place_r > 0.3501f) {
        UART_Printf("COOR_OUT_OF_REACH\r\n");
        UART_Printf("[!] Pick r=%.4f Place r=%.4f (max 0.35m)\r\n", pick_r, place_r);
        RobotUartDebug_ReportStatus(UART_STATUS_REJECTED);
        return;
    }
    
    auto_angle = angle;
    auto_pick_x = pick_x;
    auto_pick_y = pick_y;
    auto_place_x = place_x;
    auto_place_y = place_y;
    auto_has_place = 1;
    auto_state = AUTO_STATE_MOVE_XY_PICK;
    
    // Step 1: Move to pick position at z=0.08
    float32_t T_d[16];
    Convert_PTP_to_Td(auto_angle, auto_pick_x, auto_pick_y, 0.08f, T_d);
    Robot_Set_IK_Target_PD(T_d, 1);
}

void Robot_Auto_Sequence_Service(void) {
    if (auto_state == AUTO_STATE_IDLE) return;
    
    if (e_stop_active) {
        auto_state = AUTO_STATE_IDLE;
        return;
    }

    switch(auto_state) {
        // ==================== PICK PHASE ====================
        case AUTO_STATE_MOVE_XY_PICK:
            if (!Robot_Is_Moving()) {
                UART_Printf("[AUTO] Reached pick XY. Moving Z down to 0.015...\r\n");
                auto_state = AUTO_STATE_MOVE_Z_DOWN_PICK;
                
                float32_t T_d[16];
                Convert_PTP_to_Td(auto_angle, auto_pick_x, auto_pick_y, 0.0105f, T_d);
                Robot_Set_IK_Target_PD(T_d, 1);
            }
            break;
            
        case AUTO_STATE_MOVE_Z_DOWN_PICK:
            if (!Robot_Is_Moving()) {
                UART_Printf("[AUTO] Z down complete. Turning magnet ON. Waiting for Limit 5 & 6...\r\n");
                Magnet_On();
                auto_state = AUTO_STATE_WAIT_SWITCH;
            }
            break;
            
        case AUTO_STATE_WAIT_SWITCH:
            if (Limit5_IsTriggered() && Limit6_IsTriggered()) {
                UART_Printf("[AUTO] Limit 5 & 6 triggered. Delaying 200ms...\r\n");
                auto_delay_start = HAL_GetTick();
                auto_state = AUTO_STATE_DELAY_PICK;
            }
            break;
            
        case AUTO_STATE_DELAY_PICK:
            if (HAL_GetTick() - auto_delay_start >= 200) {
                UART_Printf("[AUTO] Delay complete. Moving Z up to 0.08...\r\n");
                RobotUartDebug_ReportStatus(UART_STATUS_PICKED);
                auto_state = AUTO_STATE_MOVE_Z_UP_PICK;
                
                float32_t T_d2[16];
                Convert_PTP_to_Td(auto_angle, auto_pick_x, auto_pick_y, 0.08f, T_d2);
                Robot_Set_IK_Target_PD(T_d2, 1);
            }
            break;
            
        case AUTO_STATE_MOVE_Z_UP_PICK:
            if (!Robot_Is_Moving()) {
                if (auto_has_place) {
                    // Continue to place phase
                    UART_Printf("[AUTO] Pick Z up complete. Moving to place XY (%.4f, %.4f)...\r\n",
                                auto_place_x, auto_place_y);
                    auto_state = AUTO_STATE_MOVE_XY_PLACE;
                    
                    float32_t T_d3[16];
                    Convert_PTP_to_Td(auto_angle, auto_place_x, auto_place_y, 0.08f, T_d3);
                    Robot_Set_IK_Target_PD(T_d3, 1);
                } else {
                    // Pick-only mode, done
                    UART_Printf("[AUTO] Z up complete. Pick sequence done.\r\n> ");
                    auto_state = AUTO_STATE_IDLE;
                }
            }
            break;
        
        // ==================== PLACE PHASE ====================
        case AUTO_STATE_MOVE_XY_PLACE:
            if (!Robot_Is_Moving()) {
                UART_Printf("[AUTO] Reached place XY. Moving Z down to 0.02...\r\n");
                auto_state = AUTO_STATE_MOVE_Z_DOWN_PLACE;
                
                float32_t T_d4[16];
                Convert_PTP_to_Td(auto_angle, auto_place_x, auto_place_y, 0.02f, T_d4);
                Robot_Set_IK_Target_PD(T_d4, 1);
            }
            break;
            
        case AUTO_STATE_MOVE_Z_DOWN_PLACE:
            if (!Robot_Is_Moving()) {
                UART_Printf("[AUTO] Z down at place. Turning magnet OFF...\r\n");
                Magnet_Off();
                auto_state = AUTO_STATE_MAGNET_OFF;
            }
            break;
        
        case AUTO_STATE_MAGNET_OFF:
        {
            // Move Z up immediately after magnet off
            UART_Printf("[AUTO] Moving Z up to 0.08...\r\n");
            RobotUartDebug_ReportStatus(UART_STATUS_PLACED);
            auto_state = AUTO_STATE_MOVE_Z_UP_PLACE;
            
            float32_t T_d5[16];
            Convert_PTP_to_Td(auto_angle, auto_place_x, auto_place_y, 0.08f, T_d5);
            Robot_Set_IK_Target_PD(T_d5, 1);
            break;
        }
            
        case AUTO_STATE_MOVE_Z_UP_PLACE:
            if (!Robot_Is_Moving()) {
                UART_Printf("[AUTO] Place Z up complete. Returning to home position...\r\n");
                auto_state = AUTO_STATE_MOVE_HOME;
                
                // Move all motors to angle 0 -> this is the home (encoder 0) position
                float32_t target_abs_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                extern void Multi_PD_Start(float32_t target_abs_deg[4], uint8_t param_idx);
                Multi_PD_Start(target_abs_deg, 1);
            }
            break;
        
        // ==================== RETURN HOME ====================
        case AUTO_STATE_MOVE_HOME:
            if (!Robot_Is_Moving()) {
                UART_Printf("[AUTO] Home reached. Pick-and-place sequence complete!\r\n> ");
                RobotUartDebug_ReportStatus(UART_STATUS_DONE);
                auto_state = AUTO_STATE_IDLE;
            }
            break;
            
        default:
            auto_state = AUTO_STATE_IDLE;
            break;
    }
}
