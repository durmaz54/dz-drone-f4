/*
 * motor.c
 *
 *  Created on: 9 Şub 2022
 *      Author: Abdul Samet
 */


#include "motor.h"

void motor_Init(struct motors* motors){
	HAL_TIM_PWM_Start(motors->timx, MOTOR1);
	HAL_TIM_PWM_Start(motors->timx, MOTOR2);
	HAL_TIM_PWM_Start(motors->timx, MOTOR3);
	HAL_TIM_PWM_Start(motors->timx, MOTOR4);
}


void motor_Write(struct motors* motors){
	__HAL_TIM_SetCompare(motors->timx,MOTOR1,motors->motor1);
	__HAL_TIM_SetCompare(motors->timx,MOTOR2,motors->motor2);
	__HAL_TIM_SetCompare(motors->timx,MOTOR3,motors->motor3);
	__HAL_TIM_SetCompare(motors->timx,MOTOR4,motors->motor4);
}
