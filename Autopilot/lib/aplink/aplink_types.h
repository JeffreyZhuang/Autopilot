#ifndef LIB_APLINK_APLINK_TYPES_H_
#define LIB_APLINK_APLINK_TYPES_H_

#include <stdint.h>

static constexpr uint8_t START_BYTE = 0xFE;

static constexpr uint8_t HEADER_LEN = 3;
static constexpr uint8_t FOOTER_LEN = 2;

static constexpr uint8_t MAX_PAYLOAD_LEN = 255;

static constexpr uint16_t MAX_PACKET_LEN = MAX_PAYLOAD_LEN + HEADER_LEN + FOOTER_LEN;

struct aplink_msg
{
	uint16_t checksum;
	uint8_t payload_len;
	uint8_t msg_id;
	uint8_t packet[MAX_PACKET_LEN];
	uint8_t payload[MAX_PAYLOAD_LEN];
	uint8_t packet_idx; // Index in current packet for parsing
	bool start_reading = false;
};

#endif /* LIB_APLINK_APLINK_TYPES_H_ */
