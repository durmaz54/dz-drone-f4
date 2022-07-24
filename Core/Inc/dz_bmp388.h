/*
 * dz_bmp388.h
 *
 *  Created on: 23 Tem 2022
 *      Author: abdul
 */

#ifndef INC_DZ_BMP388_H_
#define INC_DZ_BMP388_H_

#include "stm32f4xx.h"

#define BMP_I2C_ADDRESS (0x76 << 1)

#define BMP_CHIP_ID 0x00
#define BMP_ERR_REG 0x02
#define BMP_STATUS 0x03
#define BMP_DATA_0 0x04
#define BMP_DATA_1 0x05
#define BMP_DATA_2 0x06
#define BMP_DATA_3 0x07
#define BMP_DATA_4 0x08
#define BMP_DATA_5 0x09
#define BMP_SENSORTIME_0 0x0C
#define BMP_SENSORTIME_1 0x0D
#define BMP_SENSORTIME_2 0x0E
#define BMP_SENSORTIME_3 0x0F
#define BMP_EVENT 0x10
#define BMP_INT_STATUS 0x11
#define BMP_FIFO_LENGTH_0 0x12
#define BMP_FIFO_LENGTH_1 0x13
#define BMP_FIFO_DATA 0x14
#define BMP_FIFO_WTM_0 0x15
#define BMP_FIFO_WTM_1 0x16
#define BMP_FIFO_CONFIG_1 0x17
#define BMP_FIFO_CONFIG_2 0x18
#define BMP_INT_CTRL 0x19
#define BMP_IF_CONF 0x1A
#define BMP_PWR_CTRL 0x1B
#define BMP_OSR 0x1C
#define BMP_ODR 0x1D
#define BMP_CONFIG 0x1F
#define BMP_CMD 0x7E



uint8_t dz_bmp_init(I2C_HandleTypeDef *i2cx);

int32_t dz_bmp_read(I2C_HandleTypeDef *i2cx);





#endif /* INC_DZ_BMP388_H_ */
