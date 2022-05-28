/*
 * dz_pid.h
 *
 *  Created on: May 10, 2022
 *      Author: abdul
 */

#ifndef INC_DZ_PID_H_
#define INC_DZ_PID_H_

#include "stdint.h"

#define ROLL_KP (double)0.5
#define ROLL_KI (double)0.005
#define ROLL_KD (double)0.5

#define PITCH_KP 5
#define PITCH_KI 5
#define PITCH_KD 5

#define YAW_KP 5
#define YAW_KI 5
#define YAW_KD 5


int16_t pidRollCalculate(int16_t ref, int16_t imu, int16_t dt);
int16_t pidPitchCalculate(int16_t ref, int16_t imu, int16_t dt);
int16_t pidYawCalculate(int16_t ref, int16_t imu, int16_t dt);

void pidRollReset();
void pidYawReset();
void pidPitchReset();


#endif /* INC_DZ_PID_H_ */
