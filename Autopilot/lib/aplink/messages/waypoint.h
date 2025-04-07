#ifndef LIB_APLINK_MESSAGES_WAYPOINT_H_
#define LIB_APLINK_MESSAGES_WAYPOINT_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t WAYPOINT_MSG_ID = 4;

struct __attribute__((packed)) aplink_waypoint
{
	int32_t lat;
	int32_t lon;
	float alt;
};

DECLARE_APLINK_MESSAGE(aplink_waypoint, WAYPOINT_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_WAYPOINT_H_ */
