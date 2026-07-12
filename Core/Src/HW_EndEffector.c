/*
 * HW_EndEffector.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#include "HW_EndEffector.h"
#include "Custom_CLI.h" // For UART_Printf

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
