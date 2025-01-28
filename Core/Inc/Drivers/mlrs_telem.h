#ifndef INC_DRIVERS_MLRS_TELEM_H_
#define INC_DRIVERS_MLRS_TELEM_H_

#include "stm32f4xx_hal.h"
#include <cstdio>
#include <cstring>

class Mlrs_telem
{
public:
	Mlrs_telem(UART_HandleTypeDef* uart);
	void setup();
	void transmit();
	void dma_complete();
private:
	UART_HandleTypeDef* _uart;
};

#endif /* INC_DRIVERS_MLRS_TELEM_H_ */
