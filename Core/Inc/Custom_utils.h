/*
 * Custom_Utils.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_CUSTOM_UTILS_H_
#define INC_CUSTOM_UTILS_H_

#include <stdint.h>
#include "main.h"

void DWT_Delay_Init(void);
uint32_t micros(void);
void delay_us(uint32_t us);
GPIO_PinState Invert_PinState(GPIO_PinState s);

void PD_Test_Start(uint8_t motor_id, float target_deg, float Kp, float Kd);
void PD_Test_Service(void);

void Multi_PD_Start(float32_t target_abs_deg[4], uint8_t param_idx);
void Multi_PD_Service(void);
uint8_t Is_Logging_Active(void);

void Convert_PTP_to_Td(float thetaD, float pos_x, float pos_y, float pos_z, float32_t *T_d_out);


void Transfer_Test_Start(uint8_t motor_id, uint32_t freq, char dir_char, float target_deg);
void Transfer_Test_Service(void);

void Matlab_PID_Test_Start(uint8_t motor_id, char dir_char, float rel_target_deg, float kp, float kd, float time_s);
void Matlab_PID_Test_Service(void);

extern float32_t kpd_M1_1[2];
extern float32_t kpd_M1_2[2];
extern float32_t kpd_M2_1[2];
extern float32_t kpd_M2_2[2];
extern float32_t kpd_M3_1[2];
extern float32_t kpd_M3_2[2];
extern float32_t kpd_M4_1[2];
extern float32_t kpd_M4_2[2];

#endif /* INC_CUSTOM_UTILS_H_ */
