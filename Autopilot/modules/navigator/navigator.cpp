#include <modules/navigator/navigator.h>

Navigator::Navigator(HAL* hal, Plane* plane) : Module(hal, plane)
{
}

void Navigator::update()
{
	// Get target waypoint
	Waypoint target_wp = _plane->waypoints[_plane->waypoint_index];

	// Convert waypoint to north east coordinates
	double tgt_north, tgt_east;
	lat_lon_to_meters(
		_plane->home_lat,
		_plane->home_lon,
		target_wp.lat,
		target_wp.lon,
		&tgt_north,
		&tgt_east
	);

	// Check distance to waypoint to determine if waypoint reached
	float rel_east = _plane->nav_pos_east - tgt_east;
	float rel_north = _plane->nav_pos_north - tgt_north;
	float dist_to_wp = sqrtf(rel_north*rel_north + rel_east*rel_east);
	if ((dist_to_wp < get_params()->navigator.min_dist_wp) &&
		(_plane->waypoint_index < _plane->num_waypoints - 1))
	{
		_plane->waypoint_index++; // Move to next waypoint
	}
}
