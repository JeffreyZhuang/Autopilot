#ifndef LIB_APLINK_MESSAGES_LOAD_WAYPOINTS_H_
#define LIB_APLINK_MESSAGES_LOAD_WAYPOINTS_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t LOAD_WAYPOINTS_MSG_ID = 8;

struct __attribute__((packed)) aplink_load_waypoints
{
	uint8_t num_waypoints;
};

DECLARE_APLINK_MESSAGE(aplink_load_waypoints, LOAD_WAYPOINTS_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_LOAD_WAYPOINTS_H_ */
