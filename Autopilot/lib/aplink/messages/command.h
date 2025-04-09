#ifndef LIB_APLINK_MESSAGES_COMMAND_H_
#define LIB_APLINK_MESSAGES_COMMAND_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t COMMAND_MSG_ID = 9;

struct aplink_command
{
	uint8_t command_id;
};

DECLARE_APLINK_MESSAGE(aplink_command, COMMAND_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_COMMAND_H_ */
