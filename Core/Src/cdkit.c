/*
 * cdkit.c
 *
 *  Created on: Mar 4, 2022
 *      Author: abdul
 */

#include "cdkit.h"

void cdkit_read(UART_HandleTypeDef UARTx, int16_t *setpointYaw,
		int16_t *setpointPitch, int16_t *setpointRoll) {
	char data[5];

	HAL_UART_Receive(&UARTx, data, 4, 50);

	if ((data[0] == CD_WHOAMI) & ((data[1] + data[2]) == data[3])) {

		switch (data[1]) {
		case CD_PITCH:
			if(data[2] == CD_PLUS){
				//dronePitchForward(setpointYaw, setpointPitch, setpointRoll);
				*setpointPitch = 100;
			}
			else{
				//dronePitchBack(setpointYaw, setpointPitch, setpointRoll);
			}
			break;
		case CD_ROLL:
			if(data[2] == CD_PLUS){
				//droneRollRight(setpointYaw, setpointPitch, setpointRoll);
			}else{
				//droneRollLeft(setpointYaw, setpointPitch, setpointRoll);
			}

			break;
		case CD_YAW:
			if(data[2] == CD_PLUS){
				//droneYawRight(setpointYaw, setpointPitch, setpointRoll);
			}else{
				//droneYawLeft(setpointYaw, setpointPitch, setpointRoll);
			}
			break;
		default:
			break;
		}
	}

}

void droneYawRight(int16_t *setpointYaw, int16_t *setpointPitch,
		int16_t *setpointRoll){

	*setpointYaw = 5;

}
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
