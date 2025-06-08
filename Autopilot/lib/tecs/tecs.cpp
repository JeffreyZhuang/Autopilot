#include "tecs.h"

// wb: weight balance
// wb = 0: only spd
// wb = 1: balanced
// wb = 2: only alt
void TECS::calc_specific_energies(float alt_m, float vel_mps, float target_vel_mps, float target_alt_m, float wb)
{
	// Calculate specific energy
	// SPe = gh
	// SKe = 1/2 v^2
	// Ignore mass since its the energy ratio that matters
	float energy_pot = G * alt_m;
	float energy_kin = 0.5 * powf(vel_mps, 2);
	float energy_total = energy_pot + energy_kin;

	// Calculate target energy using same equations
	float target_pot = G * (-target_alt_m);
	float target_kin = 0.5 * powf(target_vel_mps, 2);
	float target_total = target_pot + target_kin;

	// Clamp total energy setpoint within allowed airspeed range
	// Prevent stall/overspeed
	float min_kin = 0.5 * powf(_param.min_spd, 2);
	float max_kin = 0.5 * powf(_param.max_spd, 2);
	target_total = clamp(target_total, energy_pot + min_kin, energy_pot + max_kin);

	// Compute energy difference setpoint and measurement
	float energy_diff_setpoint = wb * target_pot - (2.0 - wb) * target_kin;
	float energy_diff = wb * energy_pot - (2.0 - wb) * energy_kin;

	// Clamp energy balance within allowed range
	float min_diff = wb * target_pot - (2.0f - wb) * max_kin;
	float max_diff = wb * target_pot - (2.0f - wb) * min_kin;
	energy_diff_setpoint = clamp(energy_diff_setpoint, min_diff, max_diff);

	_total_energy_setpoint = target_total;
	_total_energy = energy_total;
	_energy_balance_setpoint = energy_diff_setpoint;
	_energy_balance = energy_diff;
}
