/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    UNIT_RAD,
    UNIT_DEG
} AngleUnit_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef M_PI
#define M_PI 3.14159f
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define STEP_HIGH_US              8
#define START_INTERVAL_US         1200
#define DEFAULT_TARGET_INTERVAL   250
#define ACCEL_STEP_US             8

#define DC_PWM_MAX                999
#define DEFAULT_DC_PWM            700

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
#define DC_ENCODER_SIGN           1

// ======================================================
// MOTOR 1 - DM542
// ======================================================
#define M1_STEP_PORT GPIOC
#define M1_STEP_PIN  GPIO_PIN_6

#define M1_DIR_PORT  GPIOC
#define M1_DIR_PIN   GPIO_PIN_7

#define M1_EN_PORT   GPIOB
#define M1_EN_PIN    GPIO_PIN_8
#define M1_STEP 3200

// ======================================================
// MOTOR 2 - TB6600
// ======================================================
#define M2_STEP_PORT GPIOC
#define M2_STEP_PIN  GPIO_PIN_8

#define M2_DIR_PORT  GPIOC
#define M2_DIR_PIN   GPIO_PIN_9

#define M2_EN_PORT   GPIOB
#define M2_EN_PIN    GPIO_PIN_9
#define M2_STEP 1600

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
#define M4_STEP 800



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

typedef struct
{
    const char *name;

    GPIO_TypeDef *stepPort;
    uint16_t stepPin;

    GPIO_TypeDef *dirPort;
    uint16_t dirPin;

    GPIO_TypeDef *enPort;
    uint16_t enPin;

    GPIO_PinState enRunLevel;
    GPIO_PinState dirForwardLevel;

    uint8_t running;
    uint8_t forward;
    uint8_t stepHigh;

    uint32_t lastStepUs;
    uint32_t highStartUs;
    uint32_t currentIntervalUs;
    uint32_t targetIntervalUs;

} Stepper_t;

Stepper_t motor1 = {
    .name = "M1 DM542",
    .stepPort = M1_STEP_PORT,
    .stepPin = M1_STEP_PIN,
    .dirPort = M1_DIR_PORT,
    .dirPin = M1_DIR_PIN,
    .enPort = M1_EN_PORT,
    .enPin = M1_EN_PIN,
    .enRunLevel = M1_EN_RUN,
    .dirForwardLevel = M1_DIR_FORWARD,
    .running = 0,
    .forward = 1,
    .stepHigh = 0,
    .lastStepUs = 0,
    .highStartUs = 0,
    .currentIntervalUs = START_INTERVAL_US,
    .targetIntervalUs = DEFAULT_TARGET_INTERVAL // <-- ADD THIS LINE
};

Stepper_t motor2 = {
    .name = "M2 TB6600",
    .stepPort = M2_STEP_PORT,
    .stepPin = M2_STEP_PIN,
    .dirPort = M2_DIR_PORT,
    .dirPin = M2_DIR_PIN,
    .enPort = M2_EN_PORT,
    .enPin = M2_EN_PIN,
    .enRunLevel = M2_EN_RUN,
    .dirForwardLevel = M2_DIR_FORWARD,
    .running = 0,
    .forward = 1,
    .stepHigh = 0,
    .lastStepUs = 0,
    .highStartUs = 0,
    .currentIntervalUs = START_INTERVAL_US,
    .targetIntervalUs = DEFAULT_TARGET_INTERVAL // <-- ADD THIS LINE
};

Stepper_t motor4 = {
    .name = "M4 DRV8825",
    .stepPort = M4_STEP_PORT,
    .stepPin = M4_STEP_PIN,
    .dirPort = M4_DIR_PORT,
    .dirPin = M4_DIR_PIN,
    .enPort = M4_EN_PORT,
    .enPin = M4_EN_PIN,
    .enRunLevel = M4_EN_RUN,
    .dirForwardLevel = M4_DIR_FORWARD,
    .running = 0,
    .forward = 1,
    .stepHigh = 0,
    .lastStepUs = 0,
    .highStartUs = 0,
    .currentIntervalUs = START_INTERVAL_US,
    .targetIntervalUs = DEFAULT_TARGET_INTERVAL // <-- ADD THIS LINE
};

// Encoder position
volatile int32_t enc1_pos = 0;
volatile int32_t enc2_pos = 0;
volatile int32_t enc3_pos = 0;
volatile int32_t enc4_pos = 0;

static uint32_t tim2_last = 0;
static uint16_t tim3_last = 0;
static uint16_t tim4_last = 0;

uint32_t g_step_target_interval_us = DEFAULT_TARGET_INTERVAL;
uint16_t g_dc_pwm = DEFAULT_DC_PWM;

uint8_t g_dc_running = 0;
uint8_t g_dc_forward = 1;
uint8_t g_magnet_on = 0;

char rx_cmd[32];
uint8_t rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Encoders_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

GPIO_PinState Invert_PinState(GPIO_PinState s)
{
    return (s == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

// ===============================================
// Convert Count per Revolution (CPR) to angle (CPR is encoder pulse per revolution times 4
// because current encoder reading function use both upper side and lower side of a pulse)
// ===============================================
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
void delay_us(uint32_t us) {
    // Approximate loop for an STM32 running at e.g., 72MHz or 84MHz.
    // Adjust the multiplier if the speed is wildly incorrect.
    uint32_t count = us * (SystemCoreClock / 1000000) / 5;
    while(count--) {
        __asm("nop");
    }
}

void Run_Motor_Fixed_Time_Blocking(Stepper_t* motor, uint32_t pulses, uint8_t direction, float time_seconds) {
    char msg[128];

    // Prevent divide-by-zero or negative time errors
    if (pulses == 0 || time_seconds <= 0.0f) {
        return;
    }

    // 1. Calculate the required delay per pulse in microseconds
    uint32_t total_time_us = (uint32_t)(time_seconds * 1000000.0f);
    uint32_t delay_per_pulse_us = total_time_us / pulses;

    // 2. Set direction and enable motor
    GPIO_PinState dirState;
    if (direction == 1) {
        dirState = motor->dirForwardLevel;
    } else {
        dirState = (motor->dirForwardLevel == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, dirState);
    HAL_GPIO_WritePin(motor->enPort, motor->enPin, motor->enRunLevel);

    sprintf(msg, "Running %s: %lu pulses in %.2f seconds...\r\n", motor->name, pulses, time_seconds);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    sprintf(msg, "Calculated delay: %lu us/pulse\r\n", delay_per_pulse_us);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    // Force an encoder update before moving
    Encoders_Update();

    // 3. Send exact number of pulses
    for (uint32_t i = 0; i < pulses; i++) {
        HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, GPIO_PIN_SET);
        delay_us(STEP_HIGH_US);
        HAL_GPIO_WritePin(motor->stepPort, motor->stepPin, GPIO_PIN_RESET);

        // Subtract the pulse high time from the total interval to maintain exact overall timing
        if (delay_per_pulse_us > STEP_HIGH_US) {
            delay_us(delay_per_pulse_us - STEP_HIGH_US);
        }

        // Update periodically to prevent 16-bit timer overflow (TIM3/TIM4)
        if (i % 100 == 0) {
            Encoders_Update();
        }
    }

    // Force a final update after settling
    HAL_Delay(100);
    Encoders_Update();

    sprintf(msg, "%s finished move.\r\n", motor->name);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}





// ======================================================
// LIMIT FUNCTIONS
// ======================================================

uint8_t Limit1_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM1_PORT, LIM1_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit2_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM2_PORT, LIM2_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit3_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM3_PORT, LIM3_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit4_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM4_PORT, LIM4_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit5_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM5_PORT, LIM5_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Limit6_IsTriggered(void)
{
    return HAL_GPIO_ReadPin(LIM6_PORT, LIM6_PIN) == LIMIT_TRIGGERED_LEVEL;
}

uint8_t Motor_Limit_IsTriggered(uint8_t motor_id)
{
    if (motor_id == 1) return Limit1_IsTriggered();
    if (motor_id == 2) return Limit2_IsTriggered();
    if (motor_id == 3) return Limit3_IsTriggered();
    if (motor_id == 4) return Limit4_IsTriggered();

    return 0;
}

// ======================================================
// ENCODER FUNCTIONS
// ======================================================

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

// Encoder DC: PA11 = A, PA12 = B
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

// ======================================================
// STEPPER FUNCTIONS
// ======================================================

void Stepper_Enable(Stepper_t *m)
{
    HAL_GPIO_WritePin(m->enPort, m->enPin, m->enRunLevel);
}

void Stepper_Disable(Stepper_t *m)
{
    HAL_GPIO_WritePin(m->enPort, m->enPin, Invert_PinState(m->enRunLevel));
}

void Stepper_SetDir(Stepper_t *m, uint8_t forward)
{
    GPIO_PinState level;

    if (forward)
    {
        level = m->dirForwardLevel;
    }
    else
    {
        level = Invert_PinState(m->dirForwardLevel);
    }

    HAL_GPIO_WritePin(m->dirPort, m->dirPin, level);
}

void Stepper_Stop(Stepper_t *m)
{
    m->running = 0;
    m->stepHigh = 0;

    HAL_GPIO_WritePin(m->stepPort, m->stepPin, GPIO_PIN_RESET);
    Stepper_Disable(m);

    UART_Printf("%s STOP\r\n", m->name);
}

void Stepper_Start(Stepper_t *m, uint8_t motor_id, uint8_t forward)
{
#if STOP_ON_LIMIT
    if (Motor_Limit_IsTriggered(motor_id))
    {
        UART_Printf("Cannot start M%d: LIMIT%d triggered\r\n", motor_id, motor_id);
        return;
    }
#endif

    // FIX: If the motor is already running in the requested direction, do nothing.
    // This prevents the acceleration profile and timers from being abruptly reset.
    if (m->running && m->forward == forward)
    {
        return;
    }

    Stepper_SetDir(m, forward);
    Stepper_Enable(m);

    m->running = 1;
    m->forward = forward;
    m->stepHigh = 0;
    m->currentIntervalUs = START_INTERVAL_US;
    m->lastStepUs = micros();
    m->highStartUs = micros();

    UART_Printf("%s START %s\r\n", m->name, forward ? "FORWARD" : "REVERSE");
}

void Stepper_Service(Stepper_t *m)
{
    if (!m->running)
    {
        return;
    }

    uint32_t now = micros();

    if (m->stepHigh)
    {
        if ((uint32_t)(now - m->highStartUs) >= STEP_HIGH_US)
        {
            HAL_GPIO_WritePin(m->stepPort, m->stepPin, GPIO_PIN_RESET);
            m->stepHigh = 0;
        }

        return;
    }

    if ((uint32_t)(now - m->lastStepUs) >= m->currentIntervalUs)
    {
        HAL_GPIO_WritePin(m->stepPort, m->stepPin, GPIO_PIN_SET);

        m->stepHigh = 1;
        m->highStartUs = now;
        m->lastStepUs = now;

        if (m->currentIntervalUs > m->targetIntervalUs)
		{
			if (m->currentIntervalUs > (m->targetIntervalUs + ACCEL_STEP_US))
			{
				m->currentIntervalUs -= ACCEL_STEP_US;
			}
			else
			{
				m->currentIntervalUs = m->targetIntervalUs;
			}
		}
    }
}

void Stepper_All_Stop(void)
{
    Stepper_Stop(&motor1);
    Stepper_Stop(&motor2);
    Stepper_Stop(&motor4);
}

// ======================================================
// DC MOTOR TB6612 FUNCTIONS
// ======================================================

void DC_Stop(void)
{
    g_dc_running = 0;

    HAL_GPIO_WritePin(DC_AIN1_PORT, DC_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC_AIN2_PORT, DC_AIN2_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);

    UART_Printf("M4 DC STOP\r\n");
}

void DC_Start(uint8_t forward)
{
#if STOP_ON_LIMIT
    if (Limit4_IsTriggered())
    {
        UART_Printf("Cannot start M4: LIMIT4 triggered\r\n");
        return;
    }
#endif

    g_dc_running = 1;
    g_dc_forward = forward;

    HAL_GPIO_WritePin(DC_STBY_PORT, DC_STBY_PIN, GPIO_PIN_SET);

    if (forward)
    {
        HAL_GPIO_WritePin(DC_AIN1_PORT, DC_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DC_AIN2_PORT, DC_AIN2_PIN, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(DC_AIN1_PORT, DC_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DC_AIN2_PORT, DC_AIN2_PIN, GPIO_PIN_SET);
    }

    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, g_dc_pwm);

    UART_Printf("M4 DC START %s PWM=%u\r\n",
                forward ? "FORWARD" : "REVERSE",
                g_dc_pwm);
}

// ======================================================
// MAGNET FUNCTIONS
// ======================================================

void Magnet_On(void)
{
    HAL_GPIO_WritePin(MAG_PORT, MAG_PIN, GPIO_PIN_SET);
    g_magnet_on = 1;
    UART_Printf("MAGNET ON\r\n");
}

void Magnet_Off(void)
{
    HAL_GPIO_WritePin(MAG_PORT, MAG_PIN, GPIO_PIN_RESET);
    g_magnet_on = 0;
    UART_Printf("MAGNET OFF\r\n");
}

// ======================================================
// SAFETY
// ======================================================

void Limit_Safety_Service(void)
{
#if STOP_ON_LIMIT
    if (motor1.running && Limit1_IsTriggered())
    {
        UART_Printf("LIMIT1 triggered -> stop M1\r\n");
        Stepper_Stop(&motor1);
    }

    if (motor2.running && Limit2_IsTriggered())
    {
        UART_Printf("LIMIT2 triggered -> stop M2\r\n");
        Stepper_Stop(&motor2);
    }

    if (g_dc_running && Limit3_IsTriggered())
    {
        UART_Printf("LIMIT3 triggered -> stop M3 DC\r\n");
        DC_Stop();
    }

    if (motor4.running && Limit4_IsTriggered())
    {
        UART_Printf("LIMIT4 triggered -> stop M4\r\n");
        Stepper_Stop(&motor4);
    }
#endif
}


// ======================================================
// RETURN TO HOME
// ======================================================
// Helper function to home a single motor
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

        // Use the target interval to control homing speed
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

    sprintf(msg, "%s homing complete.\r\n", motor->name);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

// Main initialization routine
void Robot_Init_State(void)
{
    UART_Printf("--- STARTING ROBOT INITIALIZATION ---\r\n");

    // Motor 1: CCW (t1 -> forward = 1), Limit ID = 1
    Home_Stepper_Motor(&motor1, 1, 1);

    // Motor 2: CW (n2 -> forward = 0), Limit ID = 2
    Home_Stepper_Motor(&motor2, 2, 0);

    // Motor 4: CCW (t4 -> forward = 1), Limit ID = 4
    Home_Stepper_Motor(&motor4, 4, 1);

    // Optional: Reset encoders now that physical zero is established
    Encoders_Reset();

    UART_Printf("--- INITIALIZATION COMPLETE ---\r\n");
}

// ======================================================
// STATUS
// ======================================================

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

// ======================================================
// COMMAND PROCESSING
// ======================================================

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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();

  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);

  // Trạng thái an toàn ban đầu
  Stepper_Disable(&motor1);
  Stepper_Disable(&motor2);
  Stepper_Disable(&motor4);

  HAL_GPIO_WritePin(DC_STBY_PORT, DC_STBY_PIN, GPIO_PIN_RESET);
  DC_Stop();

  Magnet_Off();

  Encoders_Start();

  UART_Printf("\r\nSTM32F446RE FULL ROBOT CONTROL READY\r\n");
  UART_Printf("Pinout loaded from your schematic.\r\n");

  Robot_Init_State();

  Print_Help();
  Print_Status();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      /* USER CODE END WHILE */
          UART_Command_Service();

          Encoders_Update();

          Limit_Safety_Service();

          Stepper_Service(&motor1);
          Stepper_Service(&motor2);
          Stepper_Service(&motor4);

          static uint32_t lastStatus = 0;

          if ((HAL_GetTick() - lastStatus) >= 500)
          {
              // FIX: Only print status if all stepper motors are completely stopped.
              // This prevents UART blocking from starving the motor pulses.
              if (!motor1.running && !motor2.running && !motor4.running)
              {
                  Print_Status();
              }
              lastStatus = HAL_GetTick();
          }
      /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 8;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pins : PC3 PC6 PC7 PC8
                           PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB4 PB5
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
