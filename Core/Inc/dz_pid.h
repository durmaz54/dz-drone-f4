/*
 * dz_pid.h
 *
 *  Created on: May 10, 2022
 *      Author: abdul
 */

#ifndef INC_DZ_PID_H_
#define INC_DZ_PID_H_

#include "stdint.h"

#define DELTAT (double)0.01
#define PIDMAX	100
#define PIDMIN	-100

#define YAW_ID 		5
#define PITCH_ID 	6
#define ROLL_ID 	7


void pidChange_KP(uint8_t id,	uint16_t data);
void pidChange_KI(uint8_t id,	uint16_t data);
void pidChange_KD(uint8_t id,	uint16_t data);

int16_t pidRollCalculate(int16_t ref, int16_t imu);
int16_t pidPitchCalculate(int16_t ref, int16_t imu);
int16_t pidYawCalculate(int16_t ref, int16_t imu);
int16_t pidThrottleCalculate(int16_t ref, int32_t elevation);
void pid_yawChange(int16_t* imu);
void pidRollReset();
void pidYawReset();
void pidPitchReset();
void pidThrottleReset();



#endif /* INC_DZ_PID_H_ */
