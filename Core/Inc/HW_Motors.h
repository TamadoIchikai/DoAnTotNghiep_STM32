/*
 * HW_Motors.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_HW_MOTORS_H_
#define INC_HW_MOTORS_H_

#include <stdint.h>
#include "main.h"

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

extern Stepper_t motor1;
extern Stepper_t motor2;
extern Stepper_t motor4;

extern uint8_t g_dc_running;
extern uint8_t g_dc_forward;
extern uint16_t g_dc_pwm;

extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart2; // Needed for Printf inside the motor file

void DC_Stop(void);
void DC_Start(uint8_t forward);
void Stepper_Enable(Stepper_t *m);
void Stepper_Disable(Stepper_t *m);
void Stepper_SetDir(Stepper_t *m, uint8_t forward);
void Stepper_Stop(Stepper_t *m);
void Stepper_Start(Stepper_t *m, uint8_t motor_id, uint8_t forward);
void Stepper_Service(Stepper_t *m);
void Stepper_All_Stop(void);
void Run_Motor_Fixed_Time_Blocking(Stepper_t* motor, uint32_t pulses, uint8_t direction, float time_seconds) ;

#endif /* INC_HW_MOTORS_H_ */
