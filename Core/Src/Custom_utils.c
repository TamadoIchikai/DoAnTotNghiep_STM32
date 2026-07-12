/*
 * Custom_utils.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#include "Custom_utils.h"

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

void delay_us(uint32_t us) {
    // Approximate loop for an STM32 running at e.g., 72MHz or 84MHz.
    // Adjust the multiplier if the speed is wildly incorrect.
    uint32_t count = us * (SystemCoreClock / 1000000) / 5;
    while(count--) {
        __asm("nop");
    }
}

GPIO_PinState Invert_PinState(GPIO_PinState s)
{
    return (s == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
