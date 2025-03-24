#ifndef TELEM_H_
#define TELEM_H_

#include "lib/cobs/cobs.h"
#include "lib/utils/utils.h"
#include "hal.h"
#include "parameters.h"
#include "constants.h"
#include "module.h"
#include <cstdio>
#include <cstring>

struct __attribute__((packed)) Telem_payload
{
	int16_t roll;
	int16_t pitch;
	uint16_t yaw;
	int16_t alt;
	uint16_t spd;
	int16_t alt_setpoint;
	int32_t lat;
	int32_t lon;
	float nav_north;
	float nav_east;
	uint8_t mode_id;
	uint8_t wp_idx;
	uint16_t cell_voltage;
	uint16_t battery_current;
	uint16_t battery_used;
	uint16_t autopilot_current;
	uint8_t gps_sats;
	bool gps_fix;
	uint8_t aileron;
	uint8_t elevator;
	uint8_t throttle;
};

struct __attribute__((packed)) Waypoint_payload
{
	uint8_t waypoint_index;
	uint8_t total_waypoints;
	int32_t lat;
	int32_t lon;
	int16_t alt;
};

struct __attribute__((packed)) Params_payload
{
	Parameters params;
};

struct __attribute__((packed)) Time_payload
{
	uint64_t us_since_epoch;
};

static constexpr uint8_t START_BYTE = 0; // Move to autopilot_link library

// Message identifiers
static constexpr uint8_t TELEM_MSG_ID = 1;
static constexpr uint8_t WPT_MSG_ID = 2;
static constexpr uint8_t PARAMS_MSG_ID = 3;

// Packet length constants
static constexpr uint8_t HEADER_LEN = 4;
static constexpr uint8_t MAX_PAYLOAD_LEN = 255;
static constexpr uint16_t MAX_PKT_LEN = MAX_PAYLOAD_LEN + HEADER_LEN;
static constexpr uint16_t MAX_BYTE_RATE = 1500; // Bytes per sec

class Telem : public Module
{
public:
	Telem(HAL* hal, Plane* plane);

	void update();

private:
	uint8_t _packet[MAX_PKT_LEN];
	uint16_t _pkt_idx = 0;
	bool _in_pkt = false;
	uint8_t _payload_len = 0;
	uint8_t _msg_id = 0;
	uint8_t _cobs_byte = 0;
	uint64_t _last_tlm_transmit_time = 0; // Time of last telemetry transmission
	uint16_t _bytes_since_last_tlm_transmit = 0; // Total bytes sent since last telemetry transmission
	Subscription_handle gnss_handle;
	Subscription_handle ahrs_handle;

	void transmit_packet(uint8_t packet[], uint16_t size);
	void transmit_telem(); // Transmit telemetry packet
	bool parse_packet();
	void ack();
	uint8_t get_current_state();
	Telem_payload create_telem_payload();
};

#endif /* TELEM_H_ */
