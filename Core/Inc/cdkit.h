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

void droneStop();
void droneSetHeight(int16_t maxHeight);
void droneYawRight();
void droneYawLeft();
void dronePitchForward();
void dronePitchBack();
void droneRollRight();
void droneRollLeft();
void droneSetHeight(int16_t maxHeight);

#endif /* INC_CDKIT_H_ */
