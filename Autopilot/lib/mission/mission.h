#ifndef LIB_MISSION_MISSION_H_
#define LIB_MISSION_MISSION_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_MISSION_ITEMS 256

typedef enum
{
	MISSION_EMPTY,
	MISSION_WAYPOINT,
	MISSION_LOITER,
	MISSION_LAND
} mission_type_t;

typedef enum
{
	LOITER_LEFT,
	LOITER_RIGHT
} loiter_direction_t;

typedef struct
{
	double latitude;
	double longitude;
} mission_item_t;

typedef struct {
	mission_item_t mission_items[MAX_MISSION_ITEMS];
	mission_type_t mission_type;
	uint8_t num_items;
	float loiter_radius;
	loiter_direction_t loiter_direction;
	float final_leg_dist;
	float glideslope_angle;
	float runway_heading;
	uint8_t last_item_index;
} mission_data_t;

void mission_set(const mission_data_t new_mission);
mission_data_t mission_get();
bool mission_check_loaded();
uint8_t mission_get_version();
void mission_set_altitude(float altitude);
float mission_get_altitude();

#ifdef __cplusplus
}
#endif

#endif /* LIB_MISSION_MISSION_H_ */
