/*
 * HW_Encoders.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#include "HW_Encoders.h"
#include "Custom_CLI.h" // For UART_Printf

static uint32_t tim2_last = 0;
static uint16_t tim3_last = 0;
static uint16_t tim4_last = 0;

void Encoders_Start(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    tim2_last = __HAL_TIM_GET_COUNTER(&htim2);
    tim3_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    tim4_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    enc1_pos = 0;
    enc2_pos = 0;
    enc3_pos = 0;
    enc4_pos = 0;
}

void Encoders_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    tim2_last = __HAL_TIM_GET_COUNTER(&htim2);
    tim3_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    tim4_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    enc1_pos = 0;
    enc2_pos = 0;
    enc3_pos = 0;
    enc4_pos = 0;
}

void Encoders_Update(void)
{
    uint32_t now2 = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t now3 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t now4 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    enc1_pos += (int32_t)(now2 - tim2_last);
    enc2_pos += (int16_t)(now3 - tim3_last);
    enc4_pos += (int16_t)(now4 - tim4_last);

    tim2_last = now2;
    tim3_last = now3;
    tim4_last = now4;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_11)
    {
        GPIO_PinState a = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
        GPIO_PinState b = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);

        if (a == b)
        {
            enc3_pos += DC_ENCODER_SIGN;
        }
        else
        {
            enc3_pos -= DC_ENCODER_SIGN;
        }
    }
}

float Encoder_To_Angle(int32_t counts, uint32_t ppr, AngleUnit_t unit) {
	// usage: e.g 2000 PPR encoder and variable `en1_pos` for encoder output
	// Encoder_To_Angle(en1_output, 2000, UNIT_DEG) to output degree and UNIT_RAD to output radian
    // Multiply PPR by 4 to get Counts Per Revolution (x4 decoding)
    float cpr = (float)(ppr * 4);

    // Calculate the fractional rotation
    float rotations = (float)counts / cpr;

    // Convert to the requested unit
    if (unit == UNIT_DEG) {
        return rotations * 360.0f;
    } else {
        // Default to radians
        return rotations * 2.0f * M_PI;
    }
}
