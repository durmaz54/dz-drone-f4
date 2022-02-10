/*
 * motor.h
 *
 *  Created on: 9 Şub 2022
 *      Author: Abdul Samet
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx.h"

#define MOTOR1   TIM_CHANNEL_1
#define MOTOR2   TIM_CHANNEL_2
#define MOTOR3   TIM_CHANNEL_3
#define MOTOR4   TIM_CHANNEL_4

struct motors{
	TIM_HandleTypeDef* timx;
	uint16_t motor1;
	uint16_t motor2;
	uint16_t motor3;
	uint16_t motor4;
}motors;

void motor_Init(struct motors* motors);

void motor_Write(struct motors* motors);



#endif /* INC_MOTOR_H_ */
