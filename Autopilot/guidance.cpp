#include "guidance.h"

Guidance::Guidance(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

// Load waypoints
void Guidance::init()
{
	_plane->num_waypoints = 3;
	_plane->waypoints[0] = (Waypoint){300, 500, -80};
	_plane->waypoints[1] = (Waypoint){-100, 800, -100};
	_plane->waypoints[2] = (Waypoint){-600, 800, -80};
}

// Generate position and altitude setpoint
// Detect when setpoint reached and switch to next setpoint
void Guidance::update()
{
	Waypoint current_wp = _plane->waypoints[_plane->waypoint_index];
	_plane->guidance_n_setpoint = current_wp.n;
	_plane->guidance_e_setpoint = current_wp.e;
	_plane->guidance_d_setpoint = current_wp.d;

	float err_n = _plane->guidance_n_setpoint - _plane->nav_pos_north;
	float err_e = _plane->guidance_e_setpoint - _plane->nav_pos_east;
	float dist_to_wp = sqrtf(powf(err_n, 2) + powf(err_e, 2));

	if ((dist_to_wp < MIN_DIST_WP))
	{
		_plane->waypoint_index++;
	}

	if (_plane->waypoint_index == _plane->num_waypoints)
	{
		_plane->waypoint_index = 0;
	}
}
