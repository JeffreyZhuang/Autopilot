#include "mission.h"

static uint8_t mission_version = 0;
static float mission_altitude = 0;
static bool altitude_set = false;

static mission_data_t mission = {
	.mission_type = MISSION_EMPTY,
    .num_items = 0,
    .loiter_radius = 0,
    .loiter_direction = LOITER_LEFT,
    .final_leg_dist = 0,
    .glideslope_angle = 0,
    .runway_heading = 0,
    .last_item_index = 0
};

void mission_set(const mission_data_t new_mission)
{
	mission = new_mission;
	mission_version++;
}

mission_data_t mission_get()
{
	return mission;
}

bool mission_check_loaded()
{
	return (mission.num_items > 0) && (mission.last_item_index == mission.num_items);
}

uint8_t mission_get_version()
{
	return mission_version;
}

void mission_set_altitude(float altitude)
{
	mission_altitude = altitude;
	altitude_set = true;
}

float mission_get_altitude()
{
	return mission_altitude;
}

bool mission_check_altitude_set()
{
	return altitude_set;
}
