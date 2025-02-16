#ifndef MODULES_STORAGE_STORAGE_H_
#define MODULES_STORAGE_STORAGE_H_

#include "plane.h"
#include "hal.h"
#include "Lib/COBS/cobs.h"
#include <stdint.h>
#include <cstring>

struct __attribute__((packed))Storage_payload
{
	char c[2];
	uint8_t loop_iteration;
	char a = 'a';
};

class Storage
{
public:
	Storage(Plane* plane, HAL* hal);

	void write();
	void flush();
	void read();

private:
	Plane* _plane;
	HAL* _hal;

	// Add 2 because start byte and COBS
	static constexpr int buffer_size = 100 * (sizeof(Storage_payload) + 2);
	uint8_t front_buffer[buffer_size];
	uint8_t back_buffer[buffer_size];
	bool front_buff_full = false;
	uint32_t back_buff_last_idx = 0;

};

#endif /* MODULES_STORAGE_STORAGE_H_ */
