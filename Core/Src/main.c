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

#include "arm_math.h"
#include "Custom_utils.h"
#include "Custom_CLI.h"
#include "HW_Encoders.h"
#include "HW_EndEffector.h"
#include "HW_LimitSwitches.h"
#include "HW_Motors.h"
#include "Robot_Control.h"
#include "PoE_matrixManip.h"
#include "PoE_lieTheory.h"
#include "PoE_helper.h"
#include "PoE_Kinematics.h"
#include "Controllers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


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


uint32_t g_step_target_interval_us = DEFAULT_TARGET_INTERVAL;
uint16_t g_dc_pwm = DEFAULT_DC_PWM;

uint8_t g_dc_running = 0;
uint8_t g_dc_forward = 1;
uint8_t g_magnet_on = 0;

float g_vel_filter_alpha = 0.2f;

// Re-add the encoder definitions that were removed
volatile int32_t enc1_pos = 0;
volatile int32_t enc2_pos = 0;
volatile int32_t enc3_pos = 0;
volatile int32_t enc4_pos = 0;


#define L1 0.23f
#define L2 0.12f
#define D1 0.230f
#define D2 0.049f
#define D3 0.210f
#define D_EE 0.389f

float32_t M1_Lim_Deg = 180.0f;
float32_t M2_Lim_Deg = 200.0f;
float32_t M3_Lim_Deg = 180.0f;
float32_t M4_Lim_mm  = 90.0f;
float32_t M4_Pulley_Linear_Dia = 13.0f;

float32_t robot_Init_Angle[4] = {-15.0f,38.0f,0.0f,-10.0f};
float32_t robot_Init_Pose[4] = {PI/2, -PI/2, 0, 0};

float32_t M_home_matlab[16] = {1.0f, 0.0f, 0.0f, L1+L2,
					     	   	0.0f, 1.0f, 0.0f, 0.0f,
					    		0.0f, 0.0f, 1.0f, D1+D2+D3-D_EE,
					    		0.0f, 0.0f, 0.0f, 1.0f};


float32_t S[24] = {0.0f, 0.0f, 0.0f, 0.0f,
						   	0.0f, 0.0f, 0.0f, 0.0f,
							1.0f, 1.0f, 1.0f, 0.0f,
							0.0f, 0.0f, 0.0f, 0.0f,
							0.0f, -L1, -(L1+L2), 0.0f,
							0.0f, 0.0f, 0.0f, 1.0f};
// 0.267, 0.002
// IK test target: straight arm along X axis
float32_t T_d[16] = {1.0f, 0.0f, 0.0f, (0.390f-0.124f),
                     0.0f, 1.0f, 0.0f, (0.224f-0.037f),
                     0.0f, 0.0f, 1.0f, 0.025f,
                     0.0f, 0.0f, 0.0f, 1.0f};

// IK test target: straight arm along X axis
float32_t T_d2[16] = {1.0f, 0.0f, 0.0f, 0.268f,
                     0.0f, 1.0f, 0.0f, 0.064f,
                     0.0f, 0.0f, 1.0f, D1+D2+D3-D_EE,
                     0.0f, 0.0f, 0.0f, 1.0f};

// IK test target: straight arm along X axis
float32_t T_d3[16] = {1.0f, 0.0f, 0.0f, 0.268f,
                     0.0f, 1.0f, 0.0f, -0.064f,
                     0.0f, 0.0f, 1.0f, 0.01,
                     0.0f, 0.0f, 0.0f, 1.0f};

// IK test target: straight arm along X axis
float32_t T_d4[16] = {1.0f, 0.0f, 0.0f, 0.204f,
                     0.0f, 1.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 1.0f, D1+D2+D3-D_EE,
                     0.0f, 0.0f, 0.0f, 1.0f};


float32_t kpd_M1_1[2] = {22.0f, 0.0f};
float32_t kpd_M1_2[2] = {10.0f, 0.0f};
float32_t kpd_M2_1[2] = {20.0f, 0.0f};
float32_t kpd_M2_2[2] = {20.0f, 0.0f};
float32_t kpd_M3_1[2] = {8.0f, 1.0f};
float32_t kpd_M3_2[2] = {9.6f, 1.8f};
float32_t kpd_M4_1[2] = {10.0f, 0.0f}; // Reduced from 15.0 to 10.0 to prevent oscillation
float32_t kpd_M4_2[2] = {1.0f, 0.0f};

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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PoETest(void)
{
    UART_Printf("\r\n=== Kinematics Algorithms Test ===\r\n");

    float32_t omgtheta[3] = {0.2f, 0.4f, 0.6f};
    float32_t twist[6] = {0.1f, 0.2f, 0.3f, 0.05f, 0.1f, 0.15f};
    float32_t R_in[9] = {0.0f, 0.0f, 1.0f,
                         1.0f, 0.0f, 0.0f,
                         0.0f, 1.0f, 0.0f};
    float32_t T_in[16] = {0.0f, 0.0f, 1.0f, 1.0f,
                          1.0f, 0.0f, 0.0f, 2.0f,
                          0.0f, 1.0f, 0.0f, 3.0f,
                          0.0f, 0.0f, 0.0f, 1.0f};

    float32_t omghat[3], theta;
    float32_t mat_so3[9], R_out[9];
    float32_t se3Mat[16], T_out[16];
    float32_t omgtheta_out[9], expMat_out[16];

    // 1. Vec_to_Angle
    Vec_to_Angle(omgtheta, omghat, &theta);
    UART_Printf("Vec_to_Angle:\r\n omghat=[%.4f, %.4f, %.4f], theta=%.4f\r\n\n", omghat[0], omghat[1], omghat[2], theta);

    // 2. exp_so3
    Vec_to_so3(omgtheta, mat_so3);
    exp_so3(mat_so3, R_out);
    UART_Printf("exp_so3 Output (R_out 3x3):\r\n");
    for (int i = 0; i < 3; i++) {
        UART_Printf("[ %6.4f, %6.4f, %6.4f ]\r\n", R_out[i*3], R_out[i*3+1], R_out[i*3+2]);
    }
    UART_Printf("\n");

    // 3. exp_se3
    Twist_to_se3(twist, se3Mat);
    exp_se3(se3Mat, T_out);
    UART_Printf("exp_se3 Output (T_out 4x4):\r\n");
    for (int i = 0; i < 4; i++) {
        UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n", T_out[i*4], T_out[i*4+1], T_out[i*4+2], T_out[i*4+3]);
    }
    UART_Printf("\n");

    // 4. log_SO3
    log_SO3(R_in, omgtheta_out);
    UART_Printf("log_SO3 Output (omgtheta_out 3x3):\r\n");
    for (int i = 0; i < 3; i++) {
        UART_Printf("[ %6.4f, %6.4f, %6.4f ]\r\n", omgtheta_out[i*3], omgtheta_out[i*3+1], omgtheta_out[i*3+2]);
    }
    UART_Printf("\n");

    // 5. log_SE3
    log_SE3(T_in, expMat_out);
    UART_Printf("log_SE3 Output (expMat_out 4x4):\r\n");
    for (int i = 0; i < 4; i++) {
        UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n", expMat_out[i*4], expMat_out[i*4+1], expMat_out[i*4+2], expMat_out[i*4+3]);
    }
    UART_Printf("\n");

    // 6. ForwardKinematics_Space
    float32_t M_fk[16] = {
        1.0f, 0.0f, 0.0f, 3.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    // Flat array representing a 6x3 Slist matrix
    float32_t Slist[18] = {
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,  // S1
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,  // S2
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 2.0f   // S3
    };

    float32_t thetalist[3] = {PI / 2.0f, M_PI / 4.0f, -M_PI / 4.0f};
    float32_t T_fk_out[16];

    ForwardKinematics_Space(M_fk, Slist, thetalist, 3, T_fk_out);

    UART_Printf("ForwardKinematics_Space Output (T_fk_out 4x4):\r\n");
    for (int i = 0; i < 4; i++) {
        UART_Printf("[ %6.4f, %6.4f, %6.4f, %6.4f ]\r\n", T_fk_out[i*4], T_fk_out[i*4+1], T_fk_out[i*4+2], T_fk_out[i*4+3]);
    }
    UART_Printf("\n");

    UART_Printf("====================================\r\n");
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

  // --- Initialize TIM6 for 50us (20kHz) Stepper Interrupt ---
  // SystemClock is 16MHz. 16MHz / 800 = 20kHz (50us)
  __HAL_RCC_TIM6_CLK_ENABLE();
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 799; 
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim6);
  
  // Set highest interrupt priority and enable
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  
  // Start the hardware timer interrupt
  HAL_TIM_Base_Start_IT(&htim6);

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

  Robot_Init_State(robot_Init_Angle);

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
          Software_Limit_Service();
          Encoder_Stall_Service();
          Robot_Joint_Task_Service();
          Robot_Multi_IK_Service();
          Robot_Multi_IK_PD_Service();
          PD_Test_Service();
          Multi_PD_Service();
          Transfer_Test_Service();
          Matlab_PID_Test_Service();

          UART_TX_Service();

          static uint32_t lastStatus = 0;

          if ((HAL_GetTick() - lastStatus) >= 500)
          {
              if (!Is_Logging_Active()) {
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
  htim12.Init.Period = 255;
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

  /*Configure GPIO pin : PA12 (Encoder B - also EXTI for state machine) */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
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
