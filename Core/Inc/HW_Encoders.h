/*
 * HW_Encoders.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_HW_ENCODERS_H_
#define INC_HW_ENCODERS_H_

#include <stdint.h>
#include "main.h"

typedef enum {
    UNIT_RAD,
    UNIT_DEG
} AngleUnit_t;

extern volatile int32_t enc1_pos;
extern volatile int32_t enc2_pos;
extern volatile int32_t enc3_pos;
extern volatile int32_t enc4_pos;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void Encoders_Start(void);
void Encoders_Reset(void);
void Encoders_Update(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
float Encoder_To_Angle(int32_t counts, uint32_t ppr, AngleUnit_t unit);
float Angle_To_Distance(float angle, float pulley_diameter_mm, AngleUnit_t unit);
float Encoder_To_Velocity(int32_t current_counts, int32_t previous_counts, float delta_time_sec, uint32_t ppr, AngleUnit_t unit);
float Encoder_To_Velocity_Smoothed(int32_t current_counts, int32_t previous_counts, float delta_time_sec, uint32_t ppr, AngleUnit_t unit, float *filtered_vel_state);

#endif /* INC_HW_ENCODERS_H_ */
