#include "tecs.h"

Tecs::Tecs(Plane* plane)
{
	_plane = plane;
}

// wb: weight balance
void Tecs::update(float target_vel_mps, float target_alt_m, float wb)
{
	// Calculate current energy
	// Pe = mgh
	// Ke = 1/2 mv^2
	// Ignore mass since they cancel out when computing energy ratios
	// Calculate specific energy instead
	float energy_pot = g * (-_plane->nav_pos_down);
	float energy_kin = 0.5 * powf(_plane->nav_airspeed, 2);

	// Calculate target energy using same equations
	float target_pot = g * (-target_alt_m);
	float target_kin = 0.5 * powf(target_vel_mps, 2);

	// Calculate energy error
	float err_pot = target_pot - energy_pot;
	float err_kin = target_kin - energy_kin;

	// Calculate max and min kinetic energy allowed
	float min_kin = 0.5 * powf(TECS_MIN_SPD_MPS, 2);
	float max_kin = 0.5 * powf(TECS_MAX_SPD_MPS, 2);

	// Calculate max and min errors allowed
	float min_error = min_kin - energy_kin;
	float max_error = max_kin - energy_kin;

	// Calculate total energy error and energy balance
	_plane->tecs_error_total = err_pot + err_kin;
	if (_plane->tecs_error_total > max_error)
	{
		_plane->tecs_error_total = max_error;
	}

	// wb = 1 -> balanced
	// wb = 2 -> only altitude
	// wb = 0 -> only speed
	_plane->tecs_error_diff = (2.0 - wb) * err_kin - wb * err_pot;
	_plane->tecs_error_diff = clamp(_plane->tecs_error_diff, min_error, max_error);
}
