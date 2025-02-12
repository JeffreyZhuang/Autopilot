#ifndef INC_SD_H_
#define INC_SD_H_

extern "C" {
	#include "fatfs.h"
	#include <stdio.h>
	#include <string.h>
	#include "usbd_cdc_if.h"
}

class Sd
{
public:
	void initialize();
	void write();
	void append_buffer(uint8_t* packet, uint16_t size);
	void append_byte(uint8_t byte);
	void read(uint8_t* rx_buff, uint16_t size);
private:
	FATFS fatfs;
	FIL fil;

	// Double buffering approach
	static constexpr uint32_t buffer_max_len = 2000;
	uint8_t back_buffer[buffer_max_len];
	uint8_t front_buffer[buffer_max_len];
	uint8_t new_buffer[buffer_max_len];
	bool front_buff_full = false;
	uint32_t back_buff_last_idx = 0;
};

#endif /* INC_SD_H_ */
