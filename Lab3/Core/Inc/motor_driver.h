/*
 * motor_driver.h
 *
 *  Created on: May 2, 2024
 *      Author: Aiden
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

// Motor object data structure
struct {
    int32_t  duty;
    uint32_t channel;
    TIM_TypeDef* timer;
} typedef motor_t;

// Prototype for motor object "method"
void set_duty(motor_t* p_mot, int32_t duty);
void start_PWM(TIM_HandleTypeDef* htim);
void stop_PWM(TIM_HandleTypeDef* htim);

#endif /* INC_MOTOR_H_ */
