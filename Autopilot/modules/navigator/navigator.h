#ifndef MODULES_NAVIGATOR_NAVIGATOR_H_
#define MODULES_NAVIGATOR_NAVIGATOR_H_

#include "lib/data_bus/data_bus.h"
#include "lib/parameters/params.h"
#include "lib/utils/utils.h"
#include "hal.h"
#include "module.h"

enum class MissionItemType
{
	WAYPOINT,
	LOITER,
	LAND
};

enum class LoiterDirection
{
	LEFT,
	RIGHT
};

struct MissionItem
{
	uint8_t type;
	double lat;
	double lon;
	float loiter_radius;
	uint8_t loiter_direction;
	float final_leg;
	float glideslope;
};

struct Waypoint
{
	double lat;
	double lon;
	float alt;
};

class Navigator : public Module
{
public:
	Navigator(HAL* hal, DataBus* data_bus);

	void update() override;

private:
	Subscriber<local_position_s> _local_pos_sub;
	Subscriber<telem_new_waypoint_s> _telem_new_waypoint_sub;

	Publisher<waypoint_s> _waypoint_pub;

	local_position_s _local_pos;
	telem_new_waypoint_s _telem_new_waypoint;

	uint8_t _curr_wp_idx = 1;
	Waypoint _waypoints[50];

	void poll_data_bus();
};


#endif /* MODULES_NAVIGATOR_NAVIGATOR_H_ */
