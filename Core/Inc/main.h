/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#include "arm_math.h"
extern float32_t M_home_matlab[16];
extern float32_t S[24];
extern float32_t T_d[16];
extern float32_t M4_Pulley_Linear_Dia;
extern float32_t robot_Init_Angle[4];
extern float g_vel_filter_alpha;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define M1_Err_Min 0.3f // 1600 PPR = 0.225 deg/step, must be > 0.225
#define M2_Err_Min 0.3f // 1600 PPR = 0.225 deg/step, must be > 0.225
#define M3_Err_Min 4.0f // DC motor with Kp=9.6 needs >1.04 deg to overcome 10 PWM friction deadband
#define M4_Err_Min 2.5f // 800 PPR = 0.45 deg/step, must be > 0.45
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#ifndef PI
#define PI 3.14159f
#endif

#define STEP_HIGH_US              8
#define START_INTERVAL_US         1200
#define DEFAULT_TARGET_INTERVAL   250
#define ACCEL_STEP_US             8

#define DC_PWM_MAX                999
#define DEFAULT_DC_PWM            50

#define AUTO_STATUS_ENABLE        1
#define STATUS_PERIOD_MS          700

// Nếu chưa đấu công tắc hành trình thì để 0.
// Nếu đã đấu NC + Pull-up đúng thì để 1.
#define STOP_ON_LIMIT             1

// Encoder Omron 2000 P/R.
// STM32 đọc Encoder Mode thường là x4.
#define ENCODER_PPR               2000
#define ENCODER_CPR_X4            (ENCODER_PPR * 4)

// Nếu encoder DC đếm ngược chiều mong muốn, đổi 1 thành -1.
#define DC_ENCODER_SIGN           -1

// ======================================================
// MOTOR 1 - DM542
// ======================================================
#define M1_STEP_PORT GPIOC
#define M1_STEP_PIN  GPIO_PIN_6

#define M1_DIR_PORT  GPIOC
#define M1_DIR_PIN   GPIO_PIN_7

#define M1_EN_PORT   GPIOB
#define M1_EN_PIN    GPIO_PIN_8

#define M1_STEP_VAL 1600

// ======================================================
// MOTOR 2 - TB6600
// ======================================================
#define M2_STEP_PORT GPIOC
#define M2_STEP_PIN  GPIO_PIN_8

#define M2_DIR_PORT  GPIOC
#define M2_DIR_PIN   GPIO_PIN_9

#define M2_EN_PORT   GPIOB
#define M2_EN_PIN    GPIO_PIN_9

#define M2_STEP_VAL 1600

// ======================================================
// MOTOR 3 - DC TB6612
// ======================================================
#define DC_AIN1_PORT GPIOB
#define DC_AIN1_PIN  GPIO_PIN_0

#define DC_AIN2_PORT GPIOB
#define DC_AIN2_PIN  GPIO_PIN_1

#define DC_STBY_PORT GPIOC
#define DC_STBY_PIN  GPIO_PIN_12

// ======================================================
// MOTOR 4 - DRV8825 / A4988
// ======================================================
#define M4_STEP_PORT GPIOB
#define M4_STEP_PIN  GPIO_PIN_5

#define M4_DIR_PORT  GPIOB
#define M4_DIR_PIN   GPIO_PIN_4

#define M4_EN_PORT   GPIOA
#define M4_EN_PIN    GPIO_PIN_4

#define M4_STEP_VAL 800


// PWM: PB14 / TIM12_CH1

// ======================================================
// MAGNET
// ======================================================
#define MAG_PORT GPIOC
#define MAG_PIN  GPIO_PIN_3

// ======================================================
// LIMIT 1-6 THEO SCHEMATIC CỦA BẠN
// ======================================================
#define LIM1_PORT GPIOC
#define LIM1_PIN  GPIO_PIN_4

#define LIM2_PORT GPIOC
#define LIM2_PIN  GPIO_PIN_5

#define LIM3_PORT GPIOB
#define LIM3_PIN  GPIO_PIN_12

#define LIM4_PORT GPIOB
#define LIM4_PIN  GPIO_PIN_13

#define LIM5_PORT GPIOB
#define LIM5_PIN  GPIO_PIN_2

#define LIM6_PORT GPIOB
#define LIM6_PIN  GPIO_PIN_15

// NC + Pull-up:
// Bình thường = LOW
// Chạm / đứt dây = HIGH
#define LIMIT_TRIGGERED_LEVEL GPIO_PIN_SET

// ======================================================
// ENABLE LEVEL
// ======================================================
// DRV8825/A4988: EN LOW = chạy.
// DM542/TB6600 qua ULN: tạm để LOW.
// Nếu DM542 hoặc TB6600 không chạy, đổi M1_EN_RUN/M2_EN_RUN thành GPIO_PIN_SET.

#define M1_EN_RUN GPIO_PIN_RESET
#define M2_EN_RUN GPIO_PIN_RESET
#define M4_EN_RUN GPIO_PIN_RESET

// Nếu motor quay ngược ý muốn thì đổi SET thành RESET ở motor đó.
#define M1_DIR_FORWARD GPIO_PIN_RESET
#define M2_DIR_FORWARD GPIO_PIN_RESET
#define M4_DIR_FORWARD GPIO_PIN_RESET
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
