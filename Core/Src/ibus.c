/*
 * ibus.c
 *
 *  Created on: 2 Şub 2022
 *      Author: Abdul Samet
 */
#include "ibus.h"

void ibus_read(UART_HandleTypeDef* uartx, uint16_t* data){
	uint8_t rxData[32];
	HAL_StatusTypeDef state;

	state = HAL_UART_Receive(uartx, rxData, 32, 10);
	if(state == HAL_OK){
		if((rxData[1] == IBUS_LENGTH) && (rxData[2] == IBUS_COMMAND)){
			for (uint8_t i = 0; i < IBUS_USER_CHANNELS * 2; i+=2) {

				data[(i/2)+1] = (rxData[4+i] << 8) | rxData[3+i];
				if(data[1] >2500){
					data[0] = FAILSAFE_ACTIVE;
				}
				else{
					data[0] = FAILSAFE_OFF;
				}
			}

		}
	}

}
