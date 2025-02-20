/*
 * telem.h
 *
 *  Created on: Jan 28, 2025
 *      Author: jeffr
 */

#ifndef TELEM_H_
#define TELEM_H_

#include "Lib/COBS/cobs.h"
#include "Lib/Utils/utils.h"
#include "hal.h"
#include "parameters.h"
#include <cstdio>
#include <cstring>

static constexpr uint8_t TELEM_PKT_LEN = 40;

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

	const uint16_t max_serial_rate = 1500; // Bytes/sec
	static const uint8_t queue_len = 200;
	uint8_t latest_packet[TELEM_PKT_LEN];
	uint64_t start_time;
	uint16_t total_bytes_sent = 0;
	uint8_t queue[queue_len][TELEM_PKT_LEN];
	uint8_t current_queue_idx = 0;

	void transmit();
	void parse_telemetry();
	void acknowledgement();
	void send(uint8_t* packet, uint8_t size);
	void append_queue(uint8_t* packet, uint8_t size);
	bool arrays_are_equal(uint8_t arr1[], uint8_t arr2[], uint8_t size);
};

#endif /* TELEM_H_ */
