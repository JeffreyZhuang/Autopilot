#ifndef LIB_APLINK_MESSAGES_HITL_OUTPUT_H_
#define LIB_APLINK_MESSAGES_HITL_OUTPUT_H_

#include "aplink.h"

static constexpr uint8_t HITL_OUTPUT_MSG_ID = 4;

struct __attribute__((packed)) aplink_hitl_output
{
	uint16_t rud_duty;
	uint16_t ele_duty;
	uint16_t thr_duty;
};

bool aplink_hitl_output_msg_decode(aplink_msg msg)
{
	aplink_hitl_output hitl_output;

}

#endif /* LIB_APLINK_MESSAGES_HITL_OUTPUT_H_ */
