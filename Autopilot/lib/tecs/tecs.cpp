#include "tecs.h"

void TECS::update(float alt_m, float vel_mps, float target_alt_m, float target_vel_mps)
{
	const State state{alt_m, vel_mps};
	const Setpoint setpoint{target_alt_m, target_vel_mps};

	SpecificEnergies specific_energies = calc_specific_energies(state, setpoint, _param);

	calc_pitch_control(specific_energies, _param);
	calc_throttle_control(specific_energies, _param);
}

TECS::SpecificEnergies TECS::calc_specific_energies(const State state, const Setpoint setpoint, const Param param)
{
	SpecificEnergies specific_energies;

	// SKe = 1/2 v^2
	specific_energies.kinetic.estimate = 0.5 * powf(state.vel_mps, 2);
	specific_energies.kinetic.setpoint = 0.5 * powf(setpoint.target_vel_mps, 2);

	// SPe = gh
	specific_energies.potential.estimate = G * state.alt_m;
	specific_energies.potential.setpoint = G * setpoint.target_alt_m;

	return specific_energies;
}

TECS::ControlValues TECS::calc_energy_balance(const SpecificEnergies specific_energies, const Param param)
{
	ControlValues energy_balance;

	/*
	 * The alt_weight variable controls how speed and altitude are prioritized by the pitch demand calculation
	 * A weighting of 1 gives equal speed and altitude priority
	 * A weighting of 0 gives 100% priority to speed control
	 * A weighting of 2 gives 100% priority to altitude control
	 */
	energy_balance.estimate = specific_energies.potential.estimate * param.alt_weight -
							  specific_energies.kinetic.estimate * (2.0 - param.alt_weight);

	energy_balance.setpoint = specific_energies.potential.setpoint * param.alt_weight -
							  specific_energies.kinetic.setpoint * (2.0 - param.alt_weight);

	// Clamp total energy setpoint within allowed airspeed range to prevent stall/overspeed
	float min_kinetic_energy = 0.5 * powf(param.min_spd, 2);
	float max_kinetic_energy = 0.5 * powf(param.max_spd, 2);
	float min_energy_balance_setpoint = specific_energies.potential.setpoint * param.alt_weight - max_kinetic_energy * (2.0f - param.alt_weight);
	float max_energy_balance_setpoint = param.alt_weight * specific_energies.potential.setpoint - min_kinetic_energy * (2.0f - param.alt_weight);

	energy_balance.setpoint = clamp(energy_balance.setpoint, min_energy_balance_setpoint, max_energy_balance_setpoint);

	return energy_balance;
}

void TECS::calc_pitch_control(const SpecificEnergies specific_energies, const Param param)
{
	ControlValues energy_balance = calc_energy_balance(specific_energies, param);

	float error = energy_balance.setpoint - energy_balance.estimate;

	_pitch_setpoint = param.pitch_gain * error;
}

TECS::ControlValues TECS::calc_total_energy(const SpecificEnergies specific_energies, const Param param)
{
	ControlValues total_energy;

	total_energy.estimate = specific_energies.kinetic.estimate + specific_energies.potential.estimate;

	total_energy.setpoint = specific_energies.kinetic.setpoint + specific_energies.potential.setpoint;

	// Clamp total energy setpoint within allowed airspeed range to prevent stall/overspeed
	float min_total_energy_setpoint = specific_energies.potential.setpoint + 0.5 * powf(param.min_spd, 2);
	float max_total_energy_setpoint = specific_energies.potential.setpoint + 0.5 * powf(param.max_spd, 2);

	total_energy.setpoint = clamp(total_energy.setpoint, min_total_energy_setpoint, max_total_energy_setpoint);

	return total_energy;
}

void TECS::calc_throttle_control(const SpecificEnergies specific_energies, const Param param)
{
	ControlValues total_energy = calc_total_energy(specific_energies, param);

	float error = total_energy.setpoint - total_energy.estimate;

	_throttle_setpoint = param.throttle_gain * error;
}
