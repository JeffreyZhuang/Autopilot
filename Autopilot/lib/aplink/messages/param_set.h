#ifndef LIB_APLINK_MESSAGES_PARAM_SET_H_
#define LIB_APLINK_MESSAGES_PARAM_SET_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t PARAM_SET_MSG_ID = 5;

struct __attribute__((packed)) aplink_param_set
{
	char param_id[16];
	uint8_t data[4];
};

bool aplink_param_set_msg_decode(aplink_msg* msg, aplink_param_set* param_set)
{
	if (msg->payload_len == sizeof(param_set))
	{
		memcpy(param_set, msg->payload, msg->payload_len);
		return true;
	}
	return false;
}

#endif /* LIB_APLINK_MESSAGES_PARAM_SET_H_ */
