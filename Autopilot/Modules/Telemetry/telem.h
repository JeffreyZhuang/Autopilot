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
#include "Lib/COBS/cobs.h"
#include "Lib/Utils/utils.h"

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
	uint8_t mode_id;
	uint8_t wp_idx;
	uint8_t empty[7];
};

// 28 bytes, but padding will align to nearest 4 bytes so it will be 30 bytes, so you have to remove padding
struct __attribute__((packed))Waypoint_payload
{
	uint8_t payload_type;
	uint8_t waypoint_index;
	float lat;
	float lon;
	float alt;
	uint8_t empty[24];
};

struct __attribute__((packed))Command_payload
{
	uint8_t payload_type;
	uint8_t command;
	uint8_t empty[36];
};

class Telem
{
public:
	Telem(HAL* hal, Plane* plane);
	void update();
private:
	HAL* _hal;
	Plane* _plane;
	void transmit();
	void parse_telemetry();
	void acknowledgement();
	uint64_t prev_transmit_time;
};

#endif /* TELEM_H_ */
