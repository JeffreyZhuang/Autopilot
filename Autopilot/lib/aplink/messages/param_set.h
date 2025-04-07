#ifndef LIB_APLINK_MESSAGES_PARAM_SET_H_
#define LIB_APLINK_MESSAGES_PARAM_SET_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t PARAM_SET_MSG_ID = 5;

struct __attribute__((packed)) aplink_param_set
{
	char param_id[16];
	union
	{
		float f;
		int32_t i;
	};
	uint8_t type; // 0 for float, 1 for int32_t
};

DECLARE_APLINK_MESSAGE(aplink_param_set, PARAM_SET_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_PARAM_SET_H_ */
