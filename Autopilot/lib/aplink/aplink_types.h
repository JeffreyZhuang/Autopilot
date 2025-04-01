#ifndef LIB_APLINK_APLINK_TYPES_H_
#define LIB_APLINK_APLINK_TYPES_H_

#include <stdint.h>

static constexpr uint8_t MAX_PAYLOAD_LEN = 255;

struct aplink_msg
{
	uint16_t checksum;
	uint8_t payload_len;
	uint8_t msg_id;
	uint8_t payload[MAX_PAYLOAD_LEN];
	uint8_t packet_idx; // Index in current packet for parsing
	bool start_reading = false;
};

#endif /* LIB_APLINK_APLINK_TYPES_H_ */
