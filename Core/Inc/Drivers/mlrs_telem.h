#ifndef INC_DRIVERS_MLRS_TELEM_H_
#define INC_DRIVERS_MLRS_TELEM_H_

#include "stm32f4xx_hal.h"
#include <cstring>
#include <cstdio>

enum class Mlrs_telem_mode
{
	DETECTING_START,
	DETECTING_LEN,
	READING_PAYLOAD
};

class Mlrs_telem
{
public:
	Mlrs_telem(UART_HandleTypeDef* uart);

	void setup();
	void transmit(uint8_t tx_buff[], int len);
	void dma_complete();
	bool read(uint8_t packet[], uint16_t* size);

private:
	UART_HandleTypeDef* _uart;
	uint8_t rx_buffer[1];

	Mlrs_telem_mode mode = Mlrs_telem_mode::DETECTING_START;

	static constexpr uint8_t header_len = 4;
	static constexpr uint16_t max_packet_len = 255 + header_len; // Payload + Header (Start byte, length byte, COBS byte)
	uint8_t complete_packet[max_packet_len];
	uint8_t working_packet[max_packet_len];
	uint8_t wrking_pkt_idx = 0;
	uint8_t packet_len = 0;
	bool in_pkt = false;
	bool new_packet = false;
};

#endif /* INC_DRIVERS_MLRS_TELEM_H_ */
