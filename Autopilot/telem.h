/*
 * telem.h
 *
 *  Created on: Jan 28, 2025
 *      Author: jeffr
 */

#ifndef TELEM_H_
#define TELEM_H_

#include <cstring>
#include "hal.h"
#include "cobs.h"

struct __attribute__((packed))Telem_payload
{
	uint8_t payload_type = 0;
	float roll;
	float pitch;
	float yaw;
	float alt;
	float spd;
	float lat;
	float lon;
	uint8_t empty[9];
};

class Telem
{
public:
	Telem(HAL* hal, Plane* plane);
	void update();
private:
	HAL* _hal;
	Plane* _plane;
	static constexpr uint8_t packet_len = 40;
	void transmit();
	void acknowledgement();
};

#endif /* TELEM_H_ */
