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

#endif /* INC_CUSTOM_UTILS_H_ */
