#include "guidance.h"

Guidance::Guidance(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

// Load waypoints
void Guidance::init()
{

}

// Generate position and altitude setpoint
// Detect when setpoint reached and switch to next setpoint
void Guidance::update()
{
	_plane->guidance_n_setpoint = 0;
	_plane->guidance_e_setpoint = 0;
	_plane->guidance_d_setpoint = -50;
}
