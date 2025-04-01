#ifndef LIB_APLINK_MESSAGES_COMMAND_H_
#define LIB_APLINK_MESSAGES_COMMAND_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t COMMAND_MSG_ID = 0;

struct aplink_command
{
	uint8_t command_id;
};

#endif /* LIB_APLINK_MESSAGES_COMMAND_H_ */
