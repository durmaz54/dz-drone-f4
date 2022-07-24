/*
 * dz_bmp388.c
 *
 *  Created on: 23 Tem 2022
 *      Author: abdul
 */


#include "dz_bmp388.h"

uint8_t dz_bmp_init(I2C_HandleTypeDef *i2cx){
	uint8_t data;


	if(HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_CHIP_ID, 1, &data, 1, 10) == HAL_OK){

		HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_FIFO_CONFIG_1, 1, &data, 1, 100);

		data &= (0 << 0);

		HAL_I2C_Mem_Write(i2cx, BMP_I2C_ADDRESS, BMP_FIFO_CONFIG_1, 1, &data, 1, 100);

		HAL_Delay(10);

		//pwr
		HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_PWR_CTRL, 1, &data, 1, 100);

		data |= (1 << 4) | (1 << 5) | (1 << 0) | (1 << 1);

		HAL_I2C_Mem_Write(i2cx, BMP_I2C_ADDRESS, BMP_PWR_CTRL, 1, &data, 1, 100);

		HAL_Delay(10);

		//oversampling 8

		HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_OSR, 1, &data, 1, 100);

		data |= (1 << 1) | (1 << 0);
		data &= (0 << 2);

		HAL_I2C_Mem_Write(i2cx, BMP_I2C_ADDRESS, BMP_OSR, 1, &data, 1, 100);

		HAL_Delay(10);

		//irr filter

		HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_CONFIG, 1, &data, 1, 100);

		data |= (1 << 1);
		data &= (0 << 0 ) & (0 << 2);

		HAL_I2C_Mem_Write(i2cx, BMP_I2C_ADDRESS, BMP_CONFIG, 1, &data, 1, 100);

		HAL_Delay(10);

		return 0;
	}
	else{
		HAL_Delay(100);
		return 1;
	}


}






int32_t dz_bmp_read(I2C_HandleTypeDef *i2cx){

	uint8_t data0, data1, data2, stat;
	int32_t press;

	HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_STATUS, 1, &stat, 1, 100);

	stat &= 0x20;

	HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_ERR_REG, 1, &stat, 1, 100);
	return stat;
	if(stat != 0){


		HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_DATA_0, 1, &data0, 1, 10);
		HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_DATA_1, 1, &data1, 1, 10);
		HAL_I2C_Mem_Read(i2cx, BMP_I2C_ADDRESS, BMP_DATA_2, 1, &data2, 1, 10);

		press = (data2 << 16) | (data1 << 8) | (data0);
		stat &= (0 << 5);
		HAL_I2C_Mem_Write(i2cx, BMP_I2C_ADDRESS, BMP_STATUS, 1, &stat, 1, 10);
	}
	else {
		return 0;
	}


	return press;
}

