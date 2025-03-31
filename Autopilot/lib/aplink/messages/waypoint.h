#include "aplink.h"

static constexpr uint8_t WAYPOINT_MSG_ID = 4;

struct __attribute__((packed)) aplink_waypoint
{
	uint8_t waypoint_index;
	uint8_t total_waypoints;
	int32_t lat;
	int32_t lon;
	int16_t alt;
};
