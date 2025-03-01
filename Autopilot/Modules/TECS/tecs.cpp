#include "tecs.h"

Tecs::Tecs(Plane* plane)
{
	_plane = plane;
}

// wb: weight balance
// wb = 0: only altitude
// wb = 1: balanced
// wb = 2: only speed
void Tecs::update(float target_vel_mps, float target_alt_m, float wb)
{
	// Calculate specific energy
	// Pe = gh
	// Ke = 1/2 v^2
	// Ignore mass since they cancel out when computing energy ratios
	float energy_pot = g * (-_plane->nav_pos_down);
	float energy_kin = 0.5 * powf(_plane->nav_airspeed, 2);
	float energy_total = energy_pot + energy_kin;

	// Calculate target energy using same equations
	float target_pot = g * (-target_alt_m);
	float target_kin = 0.5 * powf(target_vel_mps, 2);
	float target_total = target_pot + target_kin;

	// Clamp total energy setpoint within allowed airspeed range
	// Prevent stall/overspeed
	float min_kin = 0.5 * powf(params.tecs_min_aspd_mps, 2);
	float max_kin = 0.5 * powf(params.tecs_max_aspd_mps, 2);
	target_total = clamp(target_total, energy_pot + min_kin, energy_pot + max_kin);

	// Compute energy difference setpoints and measurements
	float energy_diff_setpoint = wb * target_pot - (2.0 - wb) * target_kin;
	float energy_diff = wb * energy_pot - (2.0 - wb) * energy_kin;

	// Clamp energy balance within allowed range
	float min_diff = wb * energy_pot - (2.0f - wb) * max_kin;
	float max_diff = wb * energy_pot - (2.0f - wb) * min_kin;
	energy_diff_setpoint = clamp(energy_diff_setpoint, min_diff, max_diff);

	_plane->tecs_energy_total_setpoint = target_total;
	_plane->tecs_energy_total = energy_total;
	_plane->tecs_energy_diff_setpoint = energy_diff_setpoint;
	_plane->tecs_energy_diff = energy_diff;
}
