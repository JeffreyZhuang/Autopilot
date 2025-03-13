#ifndef TELEM_H_
#define TELEM_H_

#include "Lib/COBS/cobs.h"
#include "Lib/Utils/utils.h"
#include "hal.h"
#include "parameters.h"
#include "constants.h"
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
	int16_t alt_setpoint;
};

struct __attribute__((packed))Waypoint_payload
{
	uint8_t payload_type;
	uint8_t waypoint_index;
	uint8_t total_waypoints;
	float lat;
	float lon;
	float alt;
};

struct __attribute__((packed))Params_payload
{
	uint8_t payload_type;
	Parameters params;
};

// Message identifiers
static constexpr uint8_t TELEM_MSG_ID = 0;
static constexpr uint8_t WPT_MSG_ID = 1;
static constexpr uint8_t PARAMS_MSG_ID = 2;

// Packet length constants
static constexpr uint16_t TLM_PKT_LEN = sizeof(Telem_payload) + 3; // Add header
static constexpr uint16_t MAX_PKT_LEN = 255 + 3;
static constexpr uint16_t MAX_BYTE_RATE = 1500; // Bytes per sec

class Telem
{
public:
	Telem(HAL* hal, Plane* plane);
	void update();

private:
	HAL* _hal;
	Plane* _plane;
	uint8_t latest_packet[MAX_PKT_LEN];
	uint16_t latest_pkt_len = 0;
	uint64_t last_tlm_transmit_time = 0; // Time of last telemetry transmission
	uint16_t bytes_since_last_tlm_transmit = 0; // Total bytes sent since last telemetry transmission

	void transmit_packet(uint8_t packet[], uint16_t size);
	void transmit_telem(); // Transmit telemetry packet
	bool validate_packet();
	bool parse_packet();
	void ack();
	uint8_t get_current_state();
	Telem_payload create_telem_payload();
};

#endif /* TELEM_H_ */
