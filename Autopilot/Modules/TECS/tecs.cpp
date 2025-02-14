#include "tecs.h"

Tecs::Tecs(Plane* plane)
{
	_plane = plane;
}

void Tecs::update()
{
	float target_vel_mps;
	float stall_speed_mps;
	float max_speed_mps;

	// Calculate current energy
	float energy_pot = TECS_MASS_KG * 9.81 * (-_plane->nav_pos_down);
	float energy_kin = 0.5 * TECS_MASS_KG * powf(_plane->nav_airspeed, 2);

	// Calculate target energy
	float target_pot = TECS_MASS_KG * 9.81 * (-_plane->guidance_d_setpoint);
	float target_kin = 0.5 * TECS_MASS_KG * powf(target_vel_mps, 2);

	// Calculate energy error
	float err_pot = target_pot - energy_pot;
	float err_kin = target_kin - energy_kin;

	// Calculate max and min kinetic energy allowed
	float min_kin = 0.5 * TECS_MASS_KG * powf(stall_speed_mps, 2);
	float max_kin = 0.5 * TECS_MASS_KG * powf(max_speed_mps, 2);

	// Calculate max and min errors allowed
	float min_error = min_kin - energy_kin;
	float max_error = max_kin - energy_kin;

	// Calculate total energy error and energy balance
	_plane->tecs_error_total = err_pot + err_kin;
	if (_plane->tecs_error_total > max_error)
	{
		_plane->tecs_error_total = max_error;
	}

	_plane->tecs_error_diff = clamp(err_kin - err_pot, min_error, max_error);
}
