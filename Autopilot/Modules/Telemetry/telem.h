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
	int16_t roll;
	int16_t pitch;
	uint16_t yaw;
	int16_t alt;
	uint16_t spd;
	float lat;
	float lon;
	uint8_t mode_id;
	uint8_t wp_idx;
	uint8_t gps_sats;
	bool gps_fix;
	uint8_t empty[15];
};

struct __attribute__((packed))Waypoint_payload
{
	uint8_t payload_type;
	uint8_t waypoint_index;
	float lat;
	float lon;
	float alt;
	uint8_t empty[24];
};

struct __attribute__((packed))Landing_target_payload
{
	uint8_t payload_type;
	float lat;
	float lon;
	float hdg;
	uint8_t empty[25];
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
	uint8_t latest_packet[TELEM_PKT_LEN];
	uint64_t prev_transmit_time;

	void transmit();
	void parse_telemetry();
	void acknowledgement();
};

#endif /* TELEM_H_ */
