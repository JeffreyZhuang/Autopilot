#ifndef INC_DATALOGGING_H_
#define INC_DATALOGGING_H_

// Make code robust to write delays of up to 250 ms or so
// https://community.st.com/t5/stm32-mcus-products/how-to-write-data-efficiently-to-an-sd-card/td-p/637579

extern "C" {
	#include "fatfs.h"
	#include <stdio.h>
	#include <string.h>
	#include "usbd_cdc_if.h"
}

// Beware of padding to align to 4 byte memory
struct Datalogging_packet
{
	uint32_t time;
	float acc_z;
	float alt;
};
static constexpr int datalogging_packet_size = sizeof(Datalogging_packet);

class Datalogging
{
public:
	void initialize();
	void write();
	void append_buffer(Datalogging_packet p);
	void read();
private:
	FATFS fatfs;
	FIL fil;

	// Double buffering approach
	static constexpr int buffer_len = 500;
	Datalogging_packet back_buffer[buffer_len];
	Datalogging_packet front_buffer[buffer_len];
	Datalogging_packet new_buffer[buffer_len];
	bool front_buff_full = false;
	int back_buff_idx = 0;
};

#endif /* INC_DATALOGGING_H_ */
