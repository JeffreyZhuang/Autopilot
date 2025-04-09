#ifndef LIB_APLINK_MESSAGES_COMMAND_ACK_H_
#define LIB_APLINK_MESSAGES_COMMAND_ACK_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t COMMAND_ACK_MSG_ID = 9;

struct aplink_command_ack
{
	bool success;
};

DECLARE_APLINK_MESSAGE(aplink_command_ack, COMMAND_ACK_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_COMMAND_ACK_H_ */
