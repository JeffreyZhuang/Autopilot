#include "mission.h"

static mission_item_t mission_items[256];
static float mission_altitude = 0;
static uint8_t num_items = 0;
static uint8_t last_item_index = 0;

void mission_add_item(mission_item_t item)
{
	mission_items[last_item_index++] = item;
}

mission_item_t mission_get_item(uint8_t index)
{
	return mission_items[index];
}

void mission_erase()
{
	num_items = 0;
}

bool check_mission_loaded()
{
	return (num_items > 0) && (last_item_index == num_items);
}

void mission_set_altitude(float altitude)
{
	mission_altitude = altitude;
}

float mission_get_altitude()
{
	return mission_altitude;
}
