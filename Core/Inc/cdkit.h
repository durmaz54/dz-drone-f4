/*
 * cdkit.h
 *
 *  Created on: Mar 4, 2022
 *      Author: abdul
 */

#ifndef INC_CDKIT_H_
#define INC_CDKIT_H_

#include "stm32f4xx.h"
#include "stdint.h"

#define CD_WHOAMI     0xCD
#define CD_YAW        0x50
#define CD_ROLL		0x51
#define CD_PITCH		0x52
#define CD_PLUS    	0x53

void cdkit_read(UART_HandleTypeDef UARTx, int16_t *setpointYaw, int16_t *setpointPitch, int16_t *setpointRoll);

void droneYawRight(int16_t *setpointYaw, int16_t *setpointPitch,
		int16_t *setpointRoll);



void droneYawLeft(int16_t *setpointYaw, int16_t *setpointPitch,
		int16_t *setpointRoll);

void dronePitchForward(int16_t *setpointYaw, int16_t *setpointPitch,
		int16_t *setpointRoll);
void dronePitchBack(int16_t *setpointYaw, int16_t *setpointPitch,
		int16_t *setpointRoll);

void droneRollRight(int16_t *setpointYaw, int16_t *setpointPitch,
		int16_t *setpointRoll);
void droneRollLeft(int16_t *setpointYaw, int16_t *setpointPitch,
		int16_t *setpointRoll);



#endif /* INC_CDKIT_H_ */
