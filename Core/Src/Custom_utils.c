/*
 * Custom_utils.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#include "Custom_utils.h"

void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

void delay_us(uint32_t us) {
    // Approximate loop for an STM32 running at e.g., 72MHz or 84MHz.
    // Adjust the multiplier if the speed is wildly incorrect.
    uint32_t count = us * (SystemCoreClock / 1000000) / 5;
    while(count--) {
        __asm("nop");
    }
}

GPIO_PinState Invert_PinState(GPIO_PinState s)
{
    return (s == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

// Non-blocking PD Test State Machine
#include "HW_Motors.h"
#include "HW_Encoders.h"
#include "Custom_CLI.h"
#include "Controllers.h"

#include <math.h>

typedef struct {
    uint8_t active;
    uint8_t motor_id;
    float target_deg;
    float Kp;
    float Kd;
    float prev_e;
    float prev_u;
    uint32_t last_calc_time;
    uint32_t start_time;
} PD_Test_State_t;

static PD_Test_State_t pd_test = {0};

void PD_Test_Start(uint8_t motor_id, float target_deg, float Kp, float Kd) {
    pd_test.motor_id = motor_id;
    pd_test.target_deg = target_deg;
    pd_test.Kp = Kp;
    pd_test.Kd = Kd;
    pd_test.prev_e = 0;
    pd_test.prev_u = 0;
    pd_test.active = 1;
    pd_test.last_calc_time = HAL_GetTick();
    pd_test.start_time = HAL_GetTick();

    if (motor_id == 1) Stepper_Enable(&motor1);
    else if (motor_id == 2) Stepper_Enable(&motor2);
    else if (motor_id == 4) Stepper_Enable(&motor4);

    UART_Printf("\r\n[PD_START] M%d, Kp=%.2f, Kd=%.2f, Target=%.2f\r\n", motor_id, Kp, Kd, target_deg);
}

void PD_Test_Service(void) {
    if (!pd_test.active) return;

    extern uint8_t e_stop_active;
    if (e_stop_active) {
        pd_test.active = 0;
        Stepper_All_Stop();
        DC_Stop();
        UART_Printf("[PD_END] ESTOP\r\n");
        return;
    }

    if (HAL_GetTick() - pd_test.last_calc_time < 10) {
        return; // Run PD loop at ~100Hz
    }
    pd_test.last_calc_time = HAL_GetTick();
    uint32_t elapsed_time = HAL_GetTick() - pd_test.start_time;

    int32_t enc_pos_val = 0;
    uint32_t ppr = M1_STEP_VAL;
    Stepper_t *motor = NULL;

    if (pd_test.motor_id == 1) { enc_pos_val = enc1_pos; motor = &motor1; ppr = M1_STEP_VAL; }
    else if (pd_test.motor_id == 2) { enc_pos_val = enc2_pos; motor = &motor2; ppr = M2_STEP_VAL; }
    else if (pd_test.motor_id == 3) { enc_pos_val = enc3_pos; }
    else if (pd_test.motor_id == 4) { enc_pos_val = enc4_pos; motor = &motor4; ppr = M4_STEP_VAL; }

    float current_deg = Encoder_To_Angle(enc_pos_val, ENCODER_PPR, UNIT_DEG);
    float error = pd_test.target_deg - current_deg;

    if (fabsf(error) < 0.1f && fabsf(pd_test.prev_u) < 5.0f) {
        UART_Printf("[PD_DATA] %lu, %.4f, %.4f\r\n", elapsed_time, current_deg, error);
        UART_Printf("[PD_END] REACHED\r\n> ");
        pd_test.active = 0;
        if (motor) {
            motor->targetIntervalUs = 250; // DEFAULT_TARGET_INTERVAL
            Stepper_Stop(motor);
        } else {
            DC_Stop();
        }
        return;
    }

    uint8_t dir;
    if (pd_test.motor_id == 3) {
        uint16_t pwm = PD_DC_Compute(pd_test.Kp, pd_test.Kd, error, &pd_test.prev_e, &pd_test.prev_u, &dir, M3_Err_Min);
        if (pwm > 0) {
            extern uint16_t g_dc_pwm;
            g_dc_pwm = pwm;
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm);
            DC_Start(dir);
        } else {
            DC_Stop();
        }
        UART_Printf("[PD_DATA] %lu, %.4f, %.4f, %u\r\n", elapsed_time, current_deg, error, pwm);
    } else {
        float min_err = (pd_test.motor_id == 1) ? M1_Err_Min : ((pd_test.motor_id == 2) ? M2_Err_Min : M4_Err_Min);
        uint32_t interval = PD_Stepper_Compute(pd_test.Kp, pd_test.Kd, error, &pd_test.prev_e, &pd_test.prev_u, ppr, &dir, min_err);
        if (interval == 0) {
            Stepper_Stop(motor);
        } else {
            Stepper_Start(motor, pd_test.motor_id, dir);
            motor->targetIntervalUs = interval;
        }
        UART_Printf("[PD_DATA] %lu, %.4f, %.4f, %lu\r\n", elapsed_time, current_deg, error, interval);
    }
}

// ---------------------------------------------------------
// Multi-Motor PD Test State Machine for IK Trajectory
// ---------------------------------------------------------
#include "PoE_Kinematics.h"
#include "HW_Encoders.h"

void Convert_PTP_to_Td(float thetaD, float pos_x, float pos_y, float pos_z, float32_t *T_d_out) {
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
    
    float32_t thetaD_rad = thetaD * PI / 180.0f;
    float32_t c = cosf(thetaD_rad);
    float32_t s = sinf(thetaD_rad);
    
    T_d_out[0]  = T_EE[0]*c + T_EE[1]*s;
    T_d_out[1]  = T_EE[0]*(-s) + T_EE[1]*c;
    T_d_out[2]  = T_EE[2];
    T_d_out[3]  = pos_x;
    
    T_d_out[4]  = T_EE[4]*c + T_EE[5]*s;
    T_d_out[5]  = T_EE[4]*(-s) + T_EE[5]*c;
    T_d_out[6]  = T_EE[6];
    T_d_out[7]  = pos_y;
    
    T_d_out[8]  = T_EE[8]*c + T_EE[9]*s;
    T_d_out[9]  = T_EE[8]*(-s) + T_EE[9]*c;
    T_d_out[10] = T_EE[10];
    T_d_out[11] = pos_z;
    
    T_d_out[12] = 0.0f;
    T_d_out[13] = 0.0f;
    T_d_out[14] = 0.0f;
    T_d_out[15] = 1.0f;
}


uint8_t PTP_RUN_CON_FLAG = 0;
float32_t ptp_pos_D_x = 0;
float32_t ptp_pos_D_y = 0;
float32_t ptp_pos_D_z = 0;
float32_t ptp_angle_D = 0;

static PD_Test_State_t multi_pd_state[4] = {0};
uint8_t multi_pd_active = 0;
static uint32_t multi_pd_start_time = 0;
static uint32_t multi_pd_last_calc_time = 0;
static uint32_t multi_pd_stable_start = 0;

void Multi_PD_Start(float32_t target_abs_deg[4], uint8_t param_idx) {
    float32_t *kp_M1 = (param_idx == 1) ? kpd_M1_1 : kpd_M1_2;
    float32_t *kp_M2 = (param_idx == 1) ? kpd_M2_1 : kpd_M2_2;
    float32_t *kp_M3 = (param_idx == 1) ? kpd_M3_1 : kpd_M3_2;
    float32_t *kp_M4 = (param_idx == 1) ? kpd_M4_1 : kpd_M4_2;

    float32_t *kp_list[4] = {kp_M1, kp_M2, kp_M3, kp_M4};

    for (int i = 0; i < 4; i++) {
        multi_pd_state[i].motor_id = i + 1;
        multi_pd_state[i].target_deg = target_abs_deg[i];
        multi_pd_state[i].Kp = kp_list[i][0];
        multi_pd_state[i].Kd = kp_list[i][1];
        multi_pd_state[i].prev_e = 0;
        multi_pd_state[i].prev_u = 0;
        multi_pd_state[i].active = 1;
    }

    Stepper_Enable(&motor1);
    Stepper_Enable(&motor2);
    Stepper_Enable(&motor4);

    multi_pd_active = 1;
    multi_pd_start_time = HAL_GetTick();
    multi_pd_last_calc_time = HAL_GetTick();
    multi_pd_stable_start = 0;

    UART_Printf("\r\n[MPD_START] Multi-Motor PD Controller Started\r\n");
}

void Multi_PD_Service(void) {
    if (!multi_pd_active) return;

    extern uint8_t e_stop_active;
    if (e_stop_active) {
        multi_pd_active = 0;
        PTP_RUN_CON_FLAG = 0;
        Stepper_All_Stop();
        DC_Stop();
        UART_Printf("[MPD_END] ESTOP\r\n");
        return;
    }

    if (HAL_GetTick() - multi_pd_last_calc_time < 10) {
        return; // 100Hz loop
    }
    multi_pd_last_calc_time = HAL_GetTick();
    uint32_t elapsed_time = HAL_GetTick() - multi_pd_start_time;

    uint8_t all_reached = 1;
    float32_t current_angles[4] = {0};
    float32_t errors[4] = {0};
    uint32_t ctrl_signals[4] = {0};
    float32_t velocities[4] = {0};

    static int32_t prev_enc[4] = {0};
    static float32_t filtered_vels[4] = {0};
    static uint32_t prev_time = 0;
    float delta_t = (HAL_GetTick() - prev_time) / 1000.0f;
    if (delta_t <= 0.0f) delta_t = 0.01f;
    prev_time = HAL_GetTick();

    // 1. Compute control effort for all 4 motors
    for (int i = 0; i < 4; i++) {
        int32_t enc_pos_val = 0;
        uint32_t ppr = M1_STEP_VAL;
        Stepper_t *motor = NULL;

        if (i == 0) { enc_pos_val = enc1_pos; motor = &motor1; ppr = M1_STEP_VAL; }
        else if (i == 1) { enc_pos_val = enc2_pos; motor = &motor2; ppr = M2_STEP_VAL; }
        else if (i == 2) { enc_pos_val = enc3_pos; }
        else if (i == 3) { enc_pos_val = enc4_pos; motor = &motor4; ppr = M4_STEP_VAL; }

        uint32_t enc_ppr_val = (i == 2) ? 11 : ENCODER_PPR;
        current_angles[i] = Encoder_To_Angle(enc_pos_val, ENCODER_PPR, UNIT_DEG); // Kept ENCODER_PPR for angle for backwards compatibility
        
        if (elapsed_time < 15) {
            velocities[i] = Encoder_To_Velocity(enc_pos_val, prev_enc[i], delta_t, enc_ppr_val, UNIT_DEG);
            filtered_vels[i] = velocities[i];
        } else {
            velocities[i] = Encoder_To_Velocity_Smoothed(enc_pos_val, prev_enc[i], delta_t, enc_ppr_val, UNIT_DEG, &filtered_vels[i]);
        }
        prev_enc[i] = enc_pos_val;
        
        if (!multi_pd_state[i].active) {
            ctrl_signals[i] = 0;
            continue;
        }

        errors[i] = multi_pd_state[i].target_deg - current_angles[i];
        
        float min_err = 0.2f;
        if (multi_pd_state[i].motor_id == 1) min_err = M1_Err_Min;
        else if (multi_pd_state[i].motor_id == 2) min_err = M2_Err_Min;
        else if (multi_pd_state[i].motor_id == 3) min_err = M3_Err_Min;
        else if (multi_pd_state[i].motor_id == 4) min_err = M4_Err_Min;

        if (fabsf(errors[i]) > min_err || fabsf(multi_pd_state[i].prev_u) >= 5.0f) {
            all_reached = 0;
        }

        uint8_t dir;
        if (i == 2) { // DC Motor
            uint16_t pwm = PD_DC_Compute(multi_pd_state[i].Kp, multi_pd_state[i].Kd, errors[i], &multi_pd_state[i].prev_e, &multi_pd_state[i].prev_u, &dir, M3_Err_Min);
            ctrl_signals[i] = pwm;
            if (pwm > 0) {
                extern uint16_t g_dc_pwm;
                g_dc_pwm = pwm;
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm);
                DC_Start(dir);
            } else {
                DC_Stop();
            }
        } else {
            uint32_t interval = PD_Stepper_Compute(multi_pd_state[i].Kp, multi_pd_state[i].Kd, errors[i], &multi_pd_state[i].prev_e, &multi_pd_state[i].prev_u, ppr, &dir, min_err);
            ctrl_signals[i] = interval;
            if (interval == 0) {
                Stepper_Stop(motor);
            } else {
                Stepper_Start(motor, multi_pd_state[i].motor_id, dir);
                motor->targetIntervalUs = interval;
            }
        }
    }

    // 2. Compute Forward Kinematics
    extern float32_t robot_Init_Pose[4];
    extern float32_t M_home_matlab[16];
    extern float32_t S[24];
    float32_t T_EE[16];
    
    float32_t thetalist[4];
    thetalist[0] = (current_angles[0] * PI / 180.0f) + robot_Init_Pose[0];
    thetalist[1] = (current_angles[1] * PI / 180.0f) + robot_Init_Pose[1];
    thetalist[2] = (current_angles[2] * PI / 180.0f) + robot_Init_Pose[2];
    extern float32_t M4_Pulley_Linear_Dia;
    thetalist[3] = ((current_angles[3] * (PI / 180.0f) * (M4_Pulley_Linear_Dia / 2.0f)) / 1000.0f) + robot_Init_Pose[3];

    ForwardKinematics_Space(M_home_matlab, S, thetalist, 4, T_EE);

    float32_t x = T_EE[3];
    float32_t y = T_EE[7];
    float32_t z = T_EE[11];

    float32_t angle_Out = atan2f(T_EE[4], T_EE[0]) * 180.0f / PI;

    // 3. Print Telemetry
    if (PTP_RUN_CON_FLAG) {
        UART_Printf("[PTP_DATA] %lu, %.4f,%.4f,%.4f,%.4f, %.4f,%.4f,%.4f,%.4f\r\n",
                    elapsed_time, ptp_pos_D_x, ptp_pos_D_y, ptp_pos_D_z, ptp_angle_D,
                    x, y, z, angle_Out);
    } else {
        UART_Printf("[MPD_DATA] %lu, %.4f,%.4f,%.4f, %.4f,%.4f,%.4f,%lu, %.4f,%.4f,%.4f,%lu, %.4f,%.4f,%.4f,%lu, %.4f,%.4f,%.4f,%lu\r\n",
                    elapsed_time, x, y, z,
                    multi_pd_state[0].target_deg, current_angles[0], velocities[0], ctrl_signals[0],
                    multi_pd_state[1].target_deg, current_angles[1], velocities[1], ctrl_signals[1],
                    multi_pd_state[2].target_deg, current_angles[2], velocities[2], ctrl_signals[2],
                    multi_pd_state[3].target_deg, current_angles[3], velocities[3], ctrl_signals[3]);
    }

    if (all_reached) {
        if (multi_pd_stable_start == 0) {
            multi_pd_stable_start = HAL_GetTick();
        } else if (HAL_GetTick() - multi_pd_stable_start >= 500) {
            for (int i = 0; i < 4; i++) multi_pd_state[i].active = 0;
            multi_pd_active = 0;
            PTP_RUN_CON_FLAG = 0;
            Stepper_All_Stop();
            DC_Stop();
            UART_Printf("[MPD_END] REACHED\r\n> ");
            multi_pd_stable_start = 0;
        }
    } else {
        multi_pd_stable_start = 0;
    }
}

// ================= TRANSFER TEST (OPEN LOOP) =================
static struct {
    uint8_t active;
    uint8_t motor_id;
    uint32_t freq;
    uint8_t dir;
    float target_deg;
    float start_deg;
    uint32_t start_time;
    uint32_t last_calc_time;
} tf_test = {0};

// Is_Logging_Active is defined at the bottom of the file

void Transfer_Test_Start(uint8_t motor_id, uint32_t freq, char dir_char, float target_deg) {
    if (motor_id < 1 || motor_id > 4) return;
    
    tf_test.motor_id = motor_id;
    tf_test.freq = freq;
    tf_test.dir = (dir_char == 't') ? 1 : 0;
    tf_test.target_deg = target_deg;
    
    int32_t enc_pos_val = 0;
    if (motor_id == 1) enc_pos_val = enc1_pos;
    else if (motor_id == 2) enc_pos_val = enc2_pos;
    else if (motor_id == 3) enc_pos_val = enc3_pos;
    else if (motor_id == 4) enc_pos_val = enc4_pos;
    
    tf_test.start_deg = Encoder_To_Angle(enc_pos_val, ENCODER_PPR, UNIT_DEG);
    tf_test.start_time = HAL_GetTick();
    tf_test.last_calc_time = tf_test.start_time;
    tf_test.active = 1;
    
    UART_Printf("[TF_START] Motor %d Open Loop Frequency %lu Hz, Target %.2f deg\r\n", motor_id, freq, target_deg);
}

void Transfer_Test_Service(void) {
    if (!tf_test.active) return;
    if (e_stop_active) {
        tf_test.active = 0;
        UART_Printf("[TF_END] ESTOP\r\n");
        return;
    }
    
    if (HAL_GetTick() - tf_test.last_calc_time < 10) return; // 100Hz loop
    
    uint32_t elapsed_time = HAL_GetTick() - tf_test.start_time;
    float delta_t = (HAL_GetTick() - tf_test.last_calc_time) / 1000.0f;
    if (delta_t <= 0.0f) delta_t = 0.01f;
    tf_test.last_calc_time = HAL_GetTick();
    
    int32_t enc_pos_val = 0;
    Stepper_t *motor = NULL;
    uint32_t enc_ppr_val = ENCODER_PPR;
    
    if (tf_test.motor_id == 1) { enc_pos_val = enc1_pos; motor = &motor1; }
    else if (tf_test.motor_id == 2) { enc_pos_val = enc2_pos; motor = &motor2; }
    else if (tf_test.motor_id == 3) { enc_pos_val = enc3_pos; enc_ppr_val = 11; }
    else if (tf_test.motor_id == 4) { enc_pos_val = enc4_pos; motor = &motor4; }
    
    float current_deg = Encoder_To_Angle(enc_pos_val, ENCODER_PPR, UNIT_DEG);
    
    static int32_t prev_enc = 0;
    static float filtered_vel = 0.0f;
    float velocity = 0.0f;
    
    if (elapsed_time < 15) { // First loop init
        velocity = Encoder_To_Velocity(enc_pos_val, prev_enc, delta_t, enc_ppr_val, UNIT_DEG);
        filtered_vel = velocity;
    } else {
        velocity = Encoder_To_Velocity_Smoothed(enc_pos_val, prev_enc, delta_t, enc_ppr_val, UNIT_DEG, &filtered_vel);
    }
    prev_enc = enc_pos_val;
    
    // Stop condition
    if (fabsf(current_deg - tf_test.start_deg) >= tf_test.target_deg) {
        tf_test.active = 0;
        if (motor) Stepper_Stop(motor);
        else DC_Stop();
        UART_Printf("[TF_END] REACHED\r\n> ");
        return;
    }
    
    // Drive Motor
    if (motor) {
        uint32_t interval = 1000000 / tf_test.freq;
        if (interval < 200) interval = 200; // max speed limit 5kHz
        motor->targetIntervalUs = interval;
        Stepper_Start(motor, tf_test.motor_id, tf_test.dir);
    } else {
        // DC motor frequency? Meaningless, just run at PWM = freq for testing, clamp at 999
        uint16_t pwm = (tf_test.freq > 999) ? 999 : tf_test.freq;
        extern uint16_t g_dc_pwm;
        g_dc_pwm = pwm;
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm);
        DC_Start(tf_test.dir);
    }
    
    // Telemetry: Time, Input (Freq), Output (Velocity)
    UART_Printf("[TF_DATA] %lu, %lu, %.4f\r\n", elapsed_time, tf_test.freq, velocity);
}

// ================= MATLAB PID TEST =================
struct {
    uint8_t active;
    uint8_t motor_id;
    float target_deg;
    float Kp;
    float Kd;
    float prev_e;
    float prev_u;
    uint32_t start_time;
    uint32_t last_calc_time;
    uint32_t duration_ms;
} matlab_pid_test = {0};

uint8_t Is_Logging_Active(void) {
    return pd_test.active || multi_pd_active || tf_test.active || matlab_pid_test.active;
}

void Matlab_PID_Test_Start(uint8_t motor_id, char dir_char, float rel_target_deg, float kp, float kd, float time_s) {
    matlab_pid_test.duration_ms = (uint32_t)(time_s * 1000.0f);
    matlab_pid_test.motor_id = motor_id;
    matlab_pid_test.Kp = kp;
    matlab_pid_test.Kd = kd;
    
    int32_t enc_pos_val = 0;
    if (motor_id == 1) enc_pos_val = enc1_pos;
    else if (motor_id == 2) enc_pos_val = enc2_pos;
    else if (motor_id == 3) enc_pos_val = enc3_pos;
    else if (motor_id == 4) enc_pos_val = enc4_pos;

    float start_deg = Encoder_To_Angle(enc_pos_val, ENCODER_PPR, UNIT_DEG);
    
    // dir_char 'n' means negative offset, 't' means positive offset
    matlab_pid_test.target_deg = start_deg + ((dir_char == 'n') ? -rel_target_deg : rel_target_deg);
    
    matlab_pid_test.prev_e = 0;
    matlab_pid_test.prev_u = 0;
    matlab_pid_test.start_time = HAL_GetTick();
    matlab_pid_test.last_calc_time = matlab_pid_test.start_time;
    matlab_pid_test.active = 1;
    
    if (motor_id == 1) Stepper_Enable(&motor1);
    else if (motor_id == 2) Stepper_Enable(&motor2);
    else if (motor_id == 4) Stepper_Enable(&motor4);
    
    UART_Printf("\r\n[MOTOR_KP_TUNE_START] M%d, Dir=%c, Target=%.2f, Kp=%.2f, Kd=%.2f\r\n", motor_id, dir_char, matlab_pid_test.target_deg, kp, kd);
}

void Matlab_PID_Test_Service(void) {
    if (!matlab_pid_test.active) return;
    
    extern uint8_t e_stop_active;
    if (e_stop_active) {
        matlab_pid_test.active = 0;
        Stepper_All_Stop();
        DC_Stop();
        UART_Printf("[MOTOR_KP_TUNE_END] ESTOP\r\n");
        return;
    }
    
    if (HAL_GetTick() - matlab_pid_test.last_calc_time < 10) return;
    
    float delta_t = (HAL_GetTick() - matlab_pid_test.last_calc_time) / 1000.0f;
    if (delta_t <= 0) delta_t = 0.01f;
    matlab_pid_test.last_calc_time = HAL_GetTick();
    
    uint32_t elapsed_time = HAL_GetTick() - matlab_pid_test.start_time;
    
    int32_t enc_pos_val = 0;
    uint32_t ppr = M1_STEP_VAL;
    Stepper_t *motor = NULL;
    
    if (matlab_pid_test.motor_id == 1) { enc_pos_val = enc1_pos; motor = &motor1; ppr = M1_STEP_VAL; }
    else if (matlab_pid_test.motor_id == 2) { enc_pos_val = enc2_pos; motor = &motor2; ppr = M2_STEP_VAL; }
    else if (matlab_pid_test.motor_id == 3) { enc_pos_val = enc3_pos; }
    else if (matlab_pid_test.motor_id == 4) { enc_pos_val = enc4_pos; motor = &motor4; ppr = M4_STEP_VAL; }
    
    float current_deg = Encoder_To_Angle(enc_pos_val, ENCODER_PPR, UNIT_DEG);
    
    static int32_t prev_enc = 0;
    static float filtered_vel = 0.0f;
    float velocity = 0.0f;
    uint32_t enc_ppr_val = (matlab_pid_test.motor_id == 3) ? 11 : ENCODER_PPR;
    
    if (elapsed_time < 15) {
        velocity = Encoder_To_Velocity(enc_pos_val, prev_enc, delta_t, enc_ppr_val, UNIT_DEG);
        filtered_vel = velocity;
    } else {
        velocity = Encoder_To_Velocity_Smoothed(enc_pos_val, prev_enc, delta_t, enc_ppr_val, UNIT_DEG, &filtered_vel);
    }
    prev_enc = enc_pos_val;
    
    float error = matlab_pid_test.target_deg - current_deg;
    float current_time_s = elapsed_time / 1000.0f;
    
    uint8_t dir;
    float freq = 0.0f;
    uint32_t interval = 0;
    uint16_t pwm = 0;
    
    if (matlab_pid_test.motor_id == 3) {
        pwm = PD_DC_Compute(matlab_pid_test.Kp, matlab_pid_test.Kd, error, &matlab_pid_test.prev_e, &matlab_pid_test.prev_u, &dir, M3_Err_Min);
        freq = (float)pwm; // Output PWM for DC motor
    } else {
        float min_err = (matlab_pid_test.motor_id == 1) ? M1_Err_Min : ((matlab_pid_test.motor_id == 2) ? M2_Err_Min : M4_Err_Min);
        interval = PD_Stepper_Compute(matlab_pid_test.Kp, matlab_pid_test.Kd, error, &matlab_pid_test.prev_e, &matlab_pid_test.prev_u, ppr, &dir, min_err);
        if (interval > 0) {
            freq = 1000000.0f / (float)interval;
        }
    }
    
    // Output format for MATLAB: time(s), Ref angle, current encoder value (in deg), encoder velocity value (in deg/s), kp value, frequency (Hz)
    UART_Printf("[MOTOR_KP_TUNE] %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\r\n", current_time_s, matlab_pid_test.target_deg, current_deg, velocity, matlab_pid_test.Kp, freq);
    
    if (elapsed_time >= matlab_pid_test.duration_ms) {
        matlab_pid_test.active = 0;
        if (motor) {
            motor->targetIntervalUs = 250;
            Stepper_Stop(motor);
        } else {
            DC_Stop();
        }
        UART_Printf("[MOTOR_KP_TUNE_END] TIME REACHED\r\n> ");
        return;
    }
    
    if (matlab_pid_test.motor_id == 3) {
        if (pwm > 0) {
            extern uint16_t g_dc_pwm;
            g_dc_pwm = pwm;
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm);
            DC_Start(dir);
        } else {
            DC_Stop();
        }
    } else {
        if (interval == 0) {
            Stepper_Stop(motor);
        } else {
            Stepper_Start(motor, matlab_pid_test.motor_id, dir);
            motor->targetIntervalUs = interval;
        }
    }
}
