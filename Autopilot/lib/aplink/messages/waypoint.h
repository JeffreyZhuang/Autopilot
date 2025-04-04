#ifndef LIB_APLINK_MESSAGES_WAYPOINT_H_
#define LIB_APLINK_MESSAGES_WAYPOINT_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t WAYPOINT_MSG_ID = 4;

struct __attribute__((packed)) aplink_waypoint
{
	uint8_t waypoint_index;
	uint8_t num_waypoints;
	int32_t lat;
	int32_t lon;
	float alt;
};

inline bool aplink_waypoint_msg_decode(aplink_msg* msg, aplink_waypoint* waypoint)
{
	if (msg->payload_len == sizeof(aplink_waypoint))
	{
		memcpy(waypoint, msg->payload, msg->payload_len);
		return true;
	}
	return false;
}

#endif /* LIB_APLINK_MESSAGES_WAYPOINT_H_ */
