#ifndef INC_DRIVERS_CXOF_H_
#define INC_DRIVERS_CXOF_H_

#include "stm32f4xx_hal.h"
#include <cstring>

#define CXOF_HEADER         (uint8_t)0xFE
#define CXOF_FOOTER         (uint8_t)0xAA
#define CXOF_FRAME_LEN                  9

struct Cxof_frame
{
	uint8_t header;
	uint8_t len;
	uint16_t x;
	uint16_t y;
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
	void dma_complete();
	Cxof_frame result;
private:
	UART_HandleTypeDef* _uart;
	uint8_t rx_buffer[1];
	uint8_t working_frame[CXOF_FRAME_LEN];
	uint8_t frame_idx;
	bool frame_started = false;
};

#endif /* INC_DRIVERS_CXOF_H_ */
