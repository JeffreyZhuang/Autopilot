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

struct Transport_protocol_layer
{
	uint8_t start_byte = 0x00;
	uint8_t cobs;
	uint8_t payload[38];
};

struct Telem_packet
{
	float roll;
	float pitch;
	float yaw;
	float alt;
	float spd;
	float lat;
	float lon;
	bool sw;
	uint8_t footer = '\n';
};

class Telem
{
public:
	Telem(HAL* hal, Plane* plane);
	void transmit();
private:
	HAL* _hal;
	Plane* _plane;
};

#endif /* TELEM_H_ */
