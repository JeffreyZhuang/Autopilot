#ifndef INC_DRIVERS_MLRS_TELEM_H_
#define INC_DRIVERS_MLRS_TELEM_H_

#include "stm32f4xx_hal.h"
#include "cobs.h"
#include <cstdio>
#include <cstring>

class Mlrs_telem
{
public:
	static constexpr uint8_t packet_len = 40;

	Mlrs_telem(UART_HandleTypeDef* uart);
	void setup();
	void transmit(uint8_t tx_buff[], int len);
	void dma_complete();
	bool read(uint8_t packet[packet_len]);
private:
	UART_HandleTypeDef* _uart;
	uint8_t rx_buffer[1];
	uint8_t complete_packet[packet_len];
	uint8_t working_packet[packet_len];
	uint8_t packet_index = 0;
	bool new_packet = false;
};

#endif /* INC_DRIVERS_MLRS_TELEM_H_ */
