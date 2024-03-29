/*
 * ibus.h
 *
 *  Created on: 2 Şub 2022
 *      Author: Abdul Samet
 */

#ifndef INC_IBUS_H_
#define INC_IBUS_H_
#include "string.h"
#include "stm32f4xx.h"


#define IBUS_USER_CHANNELS 			10
#define IBUS_LENGTH					0x20
#define IBUS_COMMAND				0x40
#define FAILSAFE_ACTIVE				1000
#define FAILSAFE_OFF				10

void ibus_read(UART_HandleTypeDef* uartx, uint16_t* data);

#endif /* INC_IBUS_H_ */
