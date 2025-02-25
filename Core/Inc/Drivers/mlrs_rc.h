/*
 * mlrs.h
 *
 *  Created on: Dec 29, 2024
 *      Author: jeffr
 */

#ifndef INC_DRIVERS_MLRS_RC_H_
#define INC_DRIVERS_MLRS_RC_H_

#include "stm32f4xx_hal.h"
#include <cstdio>
#include "sbus.h"

class Mlrs_rc
{
public:
	Mlrs_rc(UART_HandleTypeDef* uart);
	void setup();
	void dma_complete();
	void get_rc_data(uint16_t out[], int size);
private:
	UART_HandleTypeDef* _uart;
	uint8_t _rx_buffer[1];
	uint16_t _rc_data[16];
};

#endif /* INC_DRIVERS_MLRS_RC_H_ */
