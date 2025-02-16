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
	void write(uint8_t* tx_buff, uint16_t size);
	void read(uint8_t* rx_buff, uint16_t size);
	void flush();
private:
	FATFS fatfs;
	FIL fil;
	char filename[50];
	bool closed = false;
};

//class Sd
//{
//public:
//	void initialize();
//	void write();
//	void append_buffer(uint8_t* packet, uint16_t size);
//	void append_byte(uint8_t byte);
//	void read(uint8_t* rx_buff, uint16_t size);
//private:
//	FATFS fatfs;
//	FIL fil;
//
//	char filename[50];
//
//	// Double buffering approach
//	static constexpr uint32_t buffer_max_len = 4096;
//	uint8_t back_buffer[buffer_max_len];
//	uint8_t front_buffer[buffer_max_len];
//	uint8_t new_buffer[buffer_max_len];
//	bool front_buff_full = false;
//	uint32_t back_buff_last_idx = 0;
//
//	bool closed = false; // Flag for closing the file before reading
//};

#endif /* INC_SD_H_ */
