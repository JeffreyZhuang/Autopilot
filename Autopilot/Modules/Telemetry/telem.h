#ifndef TELEM_H_
#define TELEM_H_

#include "Lib/COBS/cobs.h"
#include "Lib/Utils/utils.h"
#include "hal.h"
#include "parameters.h"
#include <cstdio>
#include <cstring>

struct __attribute__((packed))Telem_payload
{
	uint8_t payload_type;
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
};

// Payload + Header (Start byte, length byte, COBS byte)
static constexpr uint16_t telem_packet_len = sizeof(Telem_payload) + 3;

struct __attribute__((packed))Waypoint_payload
{
	uint8_t payload_type;
	uint8_t waypoint_index;
	float lat;
	float lon;
	float alt;
};

struct __attribute__((packed))Landing_target_payload
{
	uint8_t payload_type;
	float lat;
	float lon;
	float hdg;
};

struct __attribute__((packed))Command_payload
{
	uint8_t payload_type;
	uint8_t command;
};

static constexpr uint8_t CMD_MSG_ID = 1;
static constexpr uint8_t WPT_MSG_ID = 2;
static constexpr uint8_t LND_TGT_MSG_ID = 3;
static constexpr uint8_t PARAMS_MSG_ID = 4;

class Telem
{
public:
	Telem(HAL* hal, Plane* plane);
	void update();
private:
	HAL* _hal;
	Plane* _plane;

	// Payload + Header (Start byte, length byte, COBS byte)
	static constexpr uint16_t max_packet_len = 255 + 3;
	uint8_t latest_packet[max_packet_len];
	uint16_t latest_pkt_len = 0;

	uint64_t start_time = 0; // Time of first transmission
	const uint16_t max_serial_rate = 1500; // Bytes per sec
	uint64_t total_bytes_sent = 0; // Need to deal with overflow...

	void transmit_telem();
	bool parse_packet();
	void ack();
	bool compare_telem_payload(const struct Telem_payload *a, const struct Telem_payload *b);
};

#endif /* TELEM_H_ */
