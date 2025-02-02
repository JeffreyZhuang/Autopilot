#include "guidance.h"

#include <cstdio> // For testing, remove later

Guidance::Guidance(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Guidance::init()
{

}

// Generate position and altitude setpoint
// Detect when setpoint reached and switch to next setpoint
void Guidance::update_mission()
{
	// Determine target waypoint
	Waypoint target_wp = _plane->waypoints[_plane->waypoint_index];
	_plane->guidance_d_setpoint = target_wp.d;

	// Determine previous waypoint
	Waypoint prev_wp;
	if (_plane->waypoint_index > 0)
	{
		prev_wp = _plane->waypoints[_plane->waypoint_index - 1];
	}
	else
	{
		prev_wp = Waypoint{0, 0, 0};
	}

	// Calculate track heading
	float trk_hdg = atan2f(target_wp.e - prev_wp.e, target_wp.n - prev_wp.n);

	// Calculate cross track error
	float xte = cosf(trk_hdg) * (prev_wp.e - target_wp.e) - sinf(trk_hdg) * (prev_wp.e - target_wp.e);

	// Calculate heading setpoint
	_plane->guidance_hdg_setpoint = trk_hdg * 180.0f / M_PI + clamp(kP * xte, -45, 45);
	if (_plane->guidance_hdg_setpoint < 0) {
		_plane->guidance_hdg_setpoint += 360.0;
	}

	printf("%.0f %.0f\n", trk_hdg * 180.0f / M_PI, clamp(kP * xte, -45, 45));

	// Calculate distance to waypoint to determine if waypoint reached
	float err_n = target_wp.n - _plane->nav_pos_north;
	float err_e = target_wp.e - _plane->nav_pos_east;
	float dist_to_wp = sqrtf(powf(err_n, 2) + powf(err_e, 2));
	if ((dist_to_wp < MIN_DIST_WP) && (_plane->waypoint_index < _plane->num_waypoints - 1))
	{
		_plane->waypoint_index++;
	}
}

void Guidance::update_landing()
{

}
