/*
 * Custom_CLI.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_CUSTOM_CLI_H_
#define INC_CUSTOM_CLI_H_

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "arm_math.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim12;
extern float32_t M4_Pulley_Linear_Dia;

extern uint8_t e_stop_active;

void Print_Status(void);
void Print_Help(void);
void Process_Command(char *cmd);
void UART_Command_Service(void);
void UART_TX_Service(void);
void UART_Printf(const char *fmt, ...);

#endif /* INC_CUSTOM_CLI_H_ */
