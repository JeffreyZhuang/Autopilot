#ifndef LIB_APLINK_MESSAGES_TIME_H_
#define LIB_APLINK_MESSAGES_TIME_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t TIME_MSG_ID = 0;

struct __attribute__((packed)) aplink_time
{
	uint64_t us_since_epoch;
};

DECLARE_APLINK_MESSAGE(aplink_time, TIME_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_TIME_H_ */
