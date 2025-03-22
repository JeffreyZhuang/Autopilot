#include "navigator.h"

Navigator::Navigator(HAL* hal, Plane* plane)
{
	_plane = plane;
	_hal = hal;
}

void Navigator::update()
{
	// Check distance to waypoint to determine if waypoint reached
	float dist_to_wp = sqrtf(rel_north*rel_north + rel_east*rel_east);
	if ((dist_to_wp < get_params()->l1_controller.min_dist_wp) &&
		(_plane->waypoint_index < _plane->num_waypoints - 1))
	{
		_plane->waypoint_index++; // Move to next waypoint
	}
}
