#ifndef INC_SD_H_
#define INC_SD_H_

#include "ring_buffer.h"
#include <stdio.h>
#include <string.h>

extern "C" {
	#include "fatfs.h"
}

class Sd
{
public:
	Sd();

	void initialize();
	void write_byte(uint8_t byte);
	bool read(uint8_t* rx_buff, uint16_t size);
	void interrupt_callback();

private:
	FATFS fatfs;
	FIL fil;
	char filename[50];
	bool reading = false;
	ring_buffer_t ring_buffer;
};

#endif /* INC_SD_H_ */
