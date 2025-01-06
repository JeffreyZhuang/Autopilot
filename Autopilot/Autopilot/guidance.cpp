#include "guidance.h"

Guidance::Guidance(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

// Generate position and altitude setpoint
void Guidance::update()
{
	_plane->guidance_n_setpoint = 0;
	_plane->guidance_e_setpoint = 0;
	_plane->guidance_d_setpoint = 0;
}

// Compute trajectory given setpoints
