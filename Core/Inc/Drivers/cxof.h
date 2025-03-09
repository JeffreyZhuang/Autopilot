#ifndef INC_DRIVERS_CXOF_H_
#define INC_DRIVERS_CXOF_H_

#include "stm32f4xx_hal.h"
#include <cstring>
#include <stdio.h>

#define CXOF_HEADER         (uint8_t)0xFE
#define CXOF_FOOTER         (uint8_t)0xAA
#define CXOF_FRAME_LEN                  9

struct  __attribute__((packed))Cxof_frame
{
	uint8_t header;
	uint8_t len;
	int16_t x;
	int16_t y;
	uint8_t checksum;
	uint8_t surface_quality;
	uint8_t footer;
};

// PMW3901 optical flow sensor UART version, CX-OF format
class Cxof
{
public:
	Cxof(UART_HandleTypeDef* uart);
	void setup();
	bool read(Cxof_frame* frame);
	void dma_complete();
private:
	UART_HandleTypeDef* _uart;
	uint8_t rx_buffer[1];
	uint8_t working_frame[CXOF_FRAME_LEN];
	uint8_t frame_idx;
	bool frame_started = false;
	bool new_data = false;
	Cxof_frame result;
};

#endif /* INC_DRIVERS_CXOF_H_ */
