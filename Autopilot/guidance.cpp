#include "guidance.h"

Guidance::Guidance(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

// Load waypoints
void Guidance::init()
{
	Waypoint wp1;
	wp1.n = 0;
	wp1.e = 0;
	wp1.d = 0;
	waypoints[0] = wp1;

	Waypoint wp2;
	wp2.n = 0;
	wp2.e = 0;
	wp2.d = 0;
	waypoints[1] = wp2;
}

// Generate position and altitude setpoint
// Detect when setpoint reached and switch to next setpoint
void Guidance::update()
{
	_plane->guidance_n_setpoint = 0;
	_plane->guidance_e_setpoint = 0;
	_plane->guidance_d_setpoint = -300;
}
