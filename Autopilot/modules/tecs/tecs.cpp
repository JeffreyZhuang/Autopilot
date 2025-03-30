#include "modules/tecs/tecs.h"

Tecs::Tecs(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _pos_est_sub(data_bus->pos_est_node),
	  _modes_sub(data_bus->modes_node),
	  _time_sub(data_bus->time_node),
	  _rc_sub(data_bus->rc_node),
	  _l1_sub(data_bus->l1_node),
	  _tecs_pub(data_bus->tecs_node)
{
}

void Tecs::update()
{
	_modes_data = _modes_sub.get();

	if (_modes_data.system_mode == System_mode::FLIGHT)
	{
		_pos_est_data = _pos_est_sub.get();
		_rc_data = _rc_sub.get();

		switch (_modes_data.flight_mode)
		{
		case Flight_mode::MANUAL:
			handle_manual_mode();
			break;
		case Flight_mode::AUTO:
			handle_auto_mode();
			break;
		}

		_tecs_pub.publish(_tecs_data);
	}
}

void Tecs::handle_manual_mode()
{
	switch (_modes_data.manual_mode)
	{
	case Manual_mode::DIRECT:
		update_direct();
		break;
	case Manual_mode::STABILIZED:
		update_stabilized();
		break;
	}
}

void Tecs::handle_auto_mode()
{
	switch (_modes_data.auto_mode)
	{
	case Auto_mode::TAKEOFF:
		update_takeoff();
		break;
	case Auto_mode::MISSION:
		update_mission();
		break;
	case Auto_mode::LAND:
		update_land();
		break;
	case Auto_mode::FLARE:
		update_flare();
		break;
	}
}

void Tecs::update_direct()
{
	_tecs_data.pitch_setpoint = 0;
	_tecs_data.thr_cmd = _rc_data.thr_norm;
}

void Tecs::update_stabilized()
{
	_tecs_data.pitch_setpoint = _rc_data.ele_norm * get_params()->tecs.ptch_lim_deg;
	_tecs_data.thr_cmd = _rc_data.thr_norm;
}

void Tecs::update_takeoff()
{
	_tecs_data.pitch_setpoint = get_params()->takeoff.ptch;
	_tecs_data.thr_cmd = _rc_data.thr_norm;
}

void Tecs::update_mission()
{
	calculate_energies(get_params()->tecs.aspd_cruise, _l1_data.d_setpoint, 1);
	_tecs_data.pitch_setpoint = control_energy_balance();
	_tecs_data.thr_cmd = control_total_energy();
}

void Tecs::update_land()
{
	calculate_energies(get_params()->tecs.aspd_land, _l1_data.d_setpoint, 1);
	_tecs_data.pitch_setpoint = control_energy_balance();
	_tecs_data.thr_cmd = control_total_energy();
}

void Tecs::update_flare()
{
	calculate_energies(0, _l1_data.d_setpoint, 2);
	_tecs_data.pitch_setpoint = control_energy_balance();
	_tecs_data.thr_cmd = 0;
}

// wb: weight balance
// wb = 0: only spd
// wb = 1: balanced
// wb = 2: only alt
void Tecs::calculate_energies(float target_vel_mps, float target_alt_m, float wb)
{
	// Calculate specific energy
	// SPe = gh
	// SKe = 1/2 v^2
	// Ignore mass since its the energy ratio that matters
	float energy_pot = G * (-_pos_est_data.pos_d);
	float energy_kin = 0.5 * powf(_pos_est_data.gnd_spd, 2);
	float energy_total = energy_pot + energy_kin;

	// Calculate target energy using same equations
	float target_pot = G * (-target_alt_m);
	float target_kin = 0.5 * powf(target_vel_mps, 2);
	float target_total = target_pot + target_kin;

	// Clamp total energy setpoint within allowed airspeed range
	// Prevent stall/overspeed
	float min_kin = 0.5 * powf(get_params()->tecs.min_aspd_mps, 2);
	float max_kin = 0.5 * powf(get_params()->tecs.max_aspd_mps, 2);
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

float Tecs::control_energy_balance()
{
	return energy_balance_controller.get_output(
		_energy_balance,
		_energy_balance_setpoint,
		get_params()->tecs.energy_balance_kp,
		get_params()->tecs.energy_balance_ki,
		get_params()->tecs.ptch_lim_deg,
		-get_params()->tecs.ptch_lim_deg,
		get_params()->tecs.ptch_lim_deg,
		0,
		_time_data.dt_s
	);
}

float Tecs::control_total_energy()
{
	return total_energy_controller.get_output(
		_total_energy,
		_total_energy_setpoint,
		get_params()->tecs.total_energy_kp,
		get_params()->tecs.total_energy_ki,
		1,
		0,
		1,
		get_params()->perf.throttle_cruise,
		_time_data.dt_s
	);
}
