#ifndef LIB_MISSION_MISSION_H_
#define LIB_MISSION_MISSION_H_

#include <stdint.h>

typedef enum
{
	MISSION_WAYPOINT,
	MISSION_LOITER,
	MISSION_LAND
} mission_type_t;

typedef enum
{
	LEFT,
	RIGHT
} loiter_direction_t;

typedef struct
{
	mission_type_t type;
	double lat;
	double lon;
	float loiter_radius;
	loiter_direction_t loiter_direction;
	float final_leg;
	float glideslope;
} mission_item_t;

void mission_add_item(mission_item_t item);
mission_item_t mission_get_item(uint8_t index);
void mission_erase();
bool check_mission_loaded();
void mission_set_altitude(float altitude);
float mission_get_altitude();

#endif /* LIB_MISSION_MISSION_H_ */
