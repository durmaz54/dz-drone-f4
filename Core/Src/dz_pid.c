/*
 * dz_pid.c
 *
 *  Created on: May 10, 2022
 *      Author: abdul
 */


#include "dz_pid.h"

double roll_p, roll_i, roll_d;
double yaw_p, yaw_i, yaw_d;
double pitch_p, pitch_i, pitch_d;
int16_t roll_pid, yaw_pid, pitch_pid;

int16_t pidRollCalculate(int16_t ref, int16_t imu, int16_t dt){
    int16_t hata = ref - imu;
    roll_p = ROLL_KP * hata;
    roll_i = roll_i + (ROLL_KI * hata * dt);
    roll_d = ROLL_KD * (hata / dt);
    roll_pid = roll_p + roll_i + roll_d;
    return roll_pid;
}

int16_t pidPitchCalculate(int16_t ref, int16_t imu, int16_t dt){
    int16_t hata = ref - imu;
    pitch_p = PITCH_KP * hata;
    pitch_i = pitch_i + (PITCH_KI * hata * dt);
    pitch_d = PITCH_KD * (hata / dt);
    pitch_pid = pitch_p + pitch_i + pitch_d;
    return pitch_pid;
}
int16_t pidYawCalculate(int16_t ref, int16_t imu, int16_t dt){
    int16_t hata = ref - imu;
    yaw_p = YAW_KP * hata;
    yaw_i = yaw_i + (YAW_KI * hata * dt);
    yaw_d = YAW_KD * (hata / dt);
    yaw_pid = yaw_p + yaw_i + yaw_d;
    return yaw_pid;
}


void pidRollReset(){
	roll_i = 0;
}
void pidYawReset(){
	yaw_i = 0;
}
void pidPitchReset(){
	pitch_i=0;
}



