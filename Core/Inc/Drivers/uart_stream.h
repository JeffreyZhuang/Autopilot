#ifndef INC_DRIVERS_UART_STREAM_H_
#define INC_DRIVERS_UART_STREAM_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>

extern "C"
{
#include "ring_buffer.h"
}

class Uart_stream
{
public:
	Uart_stream(UART_HandleTypeDef* uart);

	void setup();
	void transmit(uint8_t tx_buff[], int len);
	bool read_byte(uint8_t* byte);
	bool buffer_empty();
	void dma_complete();

private:
	UART_HandleTypeDef* _uart;
	uint8_t rx_buffer[1];
	ring_buffer_t ring_buffer;
};

#endif /* INC_DRIVERS_UART_STREAM_H_ */
