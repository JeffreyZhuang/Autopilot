#ifndef INC_DRIVERS_MLRS_TELEM_H_
#define INC_DRIVERS_MLRS_TELEM_H_

#include "stm32f4xx_hal.h"
#include "cobs.h"
#include <cstdio>
#include <cstring>

// 28 bytes, but padding will align to nearest 4 bytes so it will be 30 bytes, so you have to remove padding
struct __attribute__((packed))Waypoint_packet
{
	uint8_t payload_type;
	uint8_t waypoint_index;
	float waypoint[3];
	uint8_t empty[24];
};

struct __attribute__((packed))Command_payload
{
	uint8_t payload_type;
	uint8_t command;
	uint8_t empty[36];
};

class Mlrs_telem
{
public:
	Mlrs_telem(UART_HandleTypeDef* uart);
	void setup();
	void transmit(uint8_t tx_buff[], int len);
	void dma_complete();
	bool read();
private:
	UART_HandleTypeDef* _uart;
	uint8_t rx_buffer[1];
	static constexpr uint8_t packet_len = 40;
	uint8_t complete_packet[packet_len];
	uint8_t working_packet[packet_len];
	uint8_t packet_index = 0;
	bool new_packet = false;
};

#endif /* INC_DRIVERS_MLRS_TELEM_H_ */
