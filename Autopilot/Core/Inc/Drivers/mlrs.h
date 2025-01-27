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
#include "sbus.h"

class MLRS
{
public:
	MLRS(UART_HandleTypeDef* uart);
	void setup();
	void dma_complete();

	static constexpr int num_channels = 6;
	uint16_t rc_data[num_channels];
private:
	UART_HandleTypeDef* _uart;
	uint8_t rx_buffer[1];

	static constexpr int frame_len = 25;
	uint8_t frame[frame_len];
	int frame_idx = 0;
};

#endif /* INC_DRIVERS_MLRS_H_ */
