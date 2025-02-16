#ifndef MODULES_STORAGE_STORAGE_H_
#define MODULES_STORAGE_STORAGE_H_

#include "plane.h"
#include "hal.h"
#include "Lib/COBS/cobs.h"
#include <stdint.h>
#include <cstring>

struct __attribute__((packed))Storage_payload
{
	uint32_t loop_iteration;
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
};

#endif /* MODULES_STORAGE_STORAGE_H_ */
