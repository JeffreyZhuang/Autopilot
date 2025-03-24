/*
 * mlrs.h
 *
 *  Created on: Dec 29, 2024
 *      Author: jeffr
 */

#ifndef INC_DRIVERS_SBUS_INPUT_H_
#define INC_DRIVERS_SBUS_INPUT_H_

#include <Drivers/sbus.h>
#include "stm32f4xx_hal.h"
#include <cstdio>

class SBUS_input
{
public:
	SBUS_input(UART_HandleTypeDef* uart);
	void setup();
	void dma_complete();
	void get_rc_data(uint16_t out[], int size);
private:
	UART_HandleTypeDef* _uart;
	uint8_t _rx_buffer[1];
	uint16_t _rc_data[16];
};

#endif /* INC_DRIVERS_SBUS_INPUT_H_ */
