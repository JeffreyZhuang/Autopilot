#ifndef LIB_APLINK_MESSAGES_HITL_OUTPUT_H_
#define LIB_APLINK_MESSAGES_HITL_OUTPUT_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t HITL_OUTPUT_MSG_ID = 4;

struct __attribute__((packed)) aplink_hitl_output
{
	uint16_t rud_duty;
	uint16_t ele_duty;
	uint16_t thr_duty;
};

// Maybe seperate function because scaling factors
inline uint16_t aplink_hitl_output_pack(aplink_hitl_output data, uint8_t packet[])
{
	uint8_t buffer[sizeof(data)];
	memcpy(&buffer, &data, sizeof(buffer));
	return aplink_pack(packet, buffer, sizeof(buffer), HITL_OUTPUT_MSG_ID);
}

#endif /* LIB_APLINK_MESSAGES_HITL_OUTPUT_H_ */
