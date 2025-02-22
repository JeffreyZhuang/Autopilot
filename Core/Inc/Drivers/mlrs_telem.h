#ifndef INC_DRIVERS_MLRS_TELEM_H_
#define INC_DRIVERS_MLRS_TELEM_H_

#include "stm32f4xx_hal.h"
#include <cstring>

class Mlrs_telem
{
public:
	Mlrs_telem(UART_HandleTypeDef* uart);
	void setup();
	void transmit(uint8_t tx_buff[], int len);
	void dma_complete();
	bool read(uint8_t packet[], uint8_t* size);
private:
	UART_HandleTypeDef* _uart;
	uint8_t rx_buffer[1];

	static constexpr uint8_t max_packet_len = 40;
	uint8_t complete_packet[max_packet_len];
	uint8_t working_packet[max_packet_len];
	uint8_t complete_packet_len = 0;
	uint8_t payload_len = 0;
	uint8_t packet_index = 0;
	bool new_packet = false;
	bool in_reading = false;
};

#endif /* INC_DRIVERS_MLRS_TELEM_H_ */
