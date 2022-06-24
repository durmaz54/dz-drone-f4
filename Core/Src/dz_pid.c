/*
 * dz_pid.c
 *
 *  Created on: May 10, 2022
 *      Author: abdul
 */

#include "dz_pid.h"

double ROLL_KP = 1;
double ROLL_KI = 0.005;
double ROLL_KD = 0.2;

double PITCH_KP = 1;
double PITCH_KI = 0.005;
double PITCH_KD = 0.2;

double YAW_KP = 2;
double YAW_KI = 0.001;
double YAW_KD = 0.0;

double roll_p, roll_i, roll_d;
int16_t previous_error_roll = 0;
int16_t previous_error_pitch = 0;
int16_t previous_error_yaw = 0;
double yaw_p, yaw_i, yaw_d;
double pitch_p, pitch_i, pitch_d;
int16_t roll_pid, yaw_pid, pitch_pid;

void pidRollChange_KP(double *data) {
	ROLL_KP = *data;
	PITCH_KP = *data;
}
void pidRollChange_KI(double *data) {
	ROLL_KI = *data;
	PITCH_KI = *data;
}
void pidRollChange_KD(double *data) {
	ROLL_KD = *data;
	PITCH_KD = *data;
}

int16_t pidRollCalculate(int16_t ref, int16_t imu, int16_t dt) {
	int16_t hata = imu - ref;
	roll_p = ROLL_KP * hata;

	if ((-10 < hata) && (hata < 10)) {
		roll_i = roll_i + (ROLL_KI * hata);
	}

	roll_d = ROLL_KD * ((hata - previous_error_roll) / dt);
	roll_pid = roll_p + roll_i + roll_d;

	previous_error_roll = hata;
	return roll_pid;
}

int16_t pidPitchCalculate(int16_t ref, int16_t imu, int16_t dt) {
	int16_t hata = imu - ref;
	pitch_p = PITCH_KP * hata;

	if ((-10 < hata) && (hata < 10)) {
		pitch_i = pitch_i + (PITCH_KI * hata);
	}

	pitch_d = PITCH_KD * ((hata - previous_error_pitch) / dt);
	pitch_pid = pitch_p + pitch_i + pitch_d;

	previous_error_pitch = hata;
	return pitch_pid;
}

int16_t pidYawCalculate(int16_t ref, int16_t imu, int16_t dt) {
	int16_t hata = imu - ref;
	yaw_p = YAW_KP * hata;

	yaw_i = yaw_i + (YAW_KI * hata);

	yaw_d = YAW_KD * ((hata - previous_error_yaw) / dt);
	yaw_pid = yaw_p + yaw_i + yaw_d;

	previous_error_yaw = hata;
	return yaw_pid;
}

void pidRollReset() {
	roll_i = 0;
}
void pidYawReset() {
	yaw_i = 0;
}
void pidPitchReset() {
	pitch_i = 0;
}

