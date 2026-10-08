#ifndef ROBOT_UART_DEBUG_H
#define ROBOT_UART_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Original Out/ API — unchanged */
void RobotUartDebug_Init(UART_HandleTypeDef *pi_uart);
void RobotUartDebug_StartReceive(void);
void RobotUartDebug_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void RobotUartDebug_Process(void);

/* === Our addition: status reporting from Robot_Control.c === */
void RobotUartDebug_ReportStatus(uint8_t status);

/* Status constants matching the binary protocol */
#define UART_STATUS_READY    0U
#define UART_STATUS_ACK      1U
#define UART_STATUS_STARTED  2U
#define UART_STATUS_PICKED   3U
#define UART_STATUS_PLACED   4U
#define UART_STATUS_DONE     5U
#define UART_STATUS_BUSY     6U
#define UART_STATUS_REJECTED 7U

#ifdef __cplusplus
}
#endif

#endif
