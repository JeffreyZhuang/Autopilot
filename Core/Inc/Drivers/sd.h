#ifndef INC_SD_H_
#define INC_SD_H_

#include "Drivers/ring_buffer.h"
#include <stdio.h>
#include <string.h>

static constexpr uint16_t BUFFER_SIZE = 4096;

extern "C" {
	#include "fatfs.h"
}

enum class SDMode
{
	IDLE,
	CREATE_FILE,
	WRITE,
	SWITCH_TO_READ,
	READ
};

class Sd
{
public:
	Sd();

	void create_file(char name[], uint8_t len);
	bool write_byte(uint8_t byte);
	bool read(uint8_t* rx_buff, uint16_t size);
	void interrupt_callback();

private:
	FATFS fatfs;
	FIL fil;
	SDMode sd_mode = SDMode::IDLE;
	ring_buffer_t ring_buffer;
	char file_name[32];
};

#endif /* INC_SD_H_ */
