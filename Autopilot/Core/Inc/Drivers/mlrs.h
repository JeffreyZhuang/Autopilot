/*
 * mlrs.h
 *
 *  Created on: Dec 29, 2024
 *      Author: jeffr
 */

#ifndef INC_DRIVERS_MLRS_H_
#define INC_DRIVERS_MLRS_H_

#include "stm32f4xx_hal.h"
#include <cstdio>

class MLRS
{
public:
	MLRS(UART_HandleTypeDef* uart);
	void setup();
	void dma_complete();
private:
	UART_HandleTypeDef* _uart;
	uint8_t rx_buffer[1];
};

#endif /* INC_DRIVERS_MLRS_H_ */
