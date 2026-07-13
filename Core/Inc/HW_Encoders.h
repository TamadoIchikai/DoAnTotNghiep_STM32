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

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

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

#endif /* INC_HW_ENCODERS_H_ */
