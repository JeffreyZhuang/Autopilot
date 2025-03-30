#ifndef LIB_AUTOPILOT_LINK_AUTOPILOT_LINK_H_
#define LIB_AUTOPILOT_LINK_AUTOPILOT_LINK_H_

#include <stdio.h>
#include <string.h>

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

// Message identifiers
static constexpr uint8_t TELEM_MSG_ID = 1;
static constexpr uint8_t WPT_MSG_ID = 2;
static constexpr uint8_t PARAMS_MSG_ID = 3;
static constexpr uint8_t HITL_MSG_ID = 4;

static constexpr uint8_t START_BYTE = 0x00;

static constexpr uint8_t HEADER_LEN = 4;
static constexpr uint8_t FOOTER_LEN = 2;

static constexpr uint8_t MAX_PAYLOAD_LEN = 255;
static constexpr uint16_t MAX_PACKET_LEN = MAX_PAYLOAD_LEN + HEADER_LEN + FOOTER_LEN;

static constexpr uint16_t CRC16_POLY = 0x8005;  // CRC-16-IBM polynomial
static constexpr uint16_t CRC16_INIT = 0xFFFF;  // Initial value

class Autopilot_link
{
public:
	uint8_t latest_packet[MAX_PACKET_LEN];
	uint8_t latest_packet_len = 0;

	bool parse_byte(uint8_t byte, uint8_t payload[], uint8_t& payload_len, uint8_t& msg_id);
	void pack(uint8_t packet[], const uint8_t payload[],
			  const uint8_t payload_len, const uint8_t msg_id);
	bool unpack(const uint8_t packet[], uint8_t payload[], uint8_t& payload_len, uint8_t& msg_id);
	uint16_t calc_packet_size(uint8_t payload_size);

private:
	uint8_t _packet[MAX_PACKET_LEN];
	uint16_t _pkt_idx;
	bool _in_pkt = false;
	uint8_t _payload_len;

	uint16_t crc16(const uint8_t data[], size_t length);
};

#endif /* LIB_AUTOPILOT_LINK_AUTOPILOT_LINK_H_ */
