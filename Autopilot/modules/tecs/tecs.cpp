#include "modules/tecs/tecs.h"

Tecs::Tecs(HAL* hal, Plane* plane)
	: Module(hal, plane),
	  energy_balance_controller(false),
	  total_energy_controller(false)
{
}

void Tecs::update()
{
	if (_plane->system_mode == System_mode::FLIGHT)
	{
		switch (_plane->flight_mode)
		{
		case Flight_mode::MANUAL:
			handle_manual_mode();
			break;
		case Flight_mode::AUTO:
			handle_auto_mode();
			break;
		}
	}
}

void Tecs::handle_manual_mode()
{
	switch (_plane->manual_mode)
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
	switch (_plane->auto_mode)
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
	case Auto_mode::TOUCHDOWN:
		update_touchdown();
		break;
	}
}

void Tecs::update_direct()
{
	_plane->thr_cmd = _plane->rc_thr_norm;
}

void Tecs::update_stabilized()
{
	_plane->pitch_setpoint = _plane->rc_ele_norm * get_params()->att_ctrl.fbw_ptch_lim;
	_plane->thr_cmd = _plane->rc_thr_norm;
}

void Tecs::update_takeoff()
{
	_plane->pitch_setpoint = get_params()->takeoff.ptch;
	_plane->thr_cmd = _plane->rc_thr_norm;
}

void Tecs::update_mission()
{
	calculate_energies(get_params()->tecs.aspd_cruise, _plane->guidance_d_setpoint, 1);
	control_energy_balance();
	control_total_energy();
}

void Tecs::update_land()
{
	calculate_energies(get_params()->tecs.aspd_land, _plane->guidance_d_setpoint, 1);
	control_energy_balance();
	control_total_energy();
}

void Tecs::update_flare()
{
	calculate_energies(0, _plane->guidance_d_setpoint, 2);
	control_energy_balance();
	_plane->thr_cmd = 0;
}

void Tecs::update_touchdown()
{
	_plane->thr_cmd = 0;
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
	float energy_pot = G * (-_plane->nav_pos_down);
	float energy_kin = 0.5 * powf(_plane->nav_gnd_spd, 2);
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

	_plane->tecs_energy_total_setpoint = target_total;
	_plane->tecs_energy_total = energy_total;
	_plane->tecs_energy_diff_setpoint = energy_diff_setpoint;
	_plane->tecs_energy_diff = energy_diff;
}

void Tecs::control_energy_balance()
{
	_plane->pitch_setpoint = energy_balance_controller.get_output(
		_plane->tecs_energy_diff,
		_plane->tecs_energy_diff_setpoint,
		get_params()->tecs.energy_balance_kp,
		get_params()->tecs.energy_balance_ki,
		get_params()->tecs.ptch_lim_deg,
		-get_params()->tecs.ptch_lim_deg,
		get_params()->tecs.ptch_lim_deg,
		0,
		_plane->dt_s
	);
}

void Tecs::control_total_energy()
{
	_plane->thr_cmd = total_energy_controller.get_output(
		_plane->tecs_energy_total,
		_plane->tecs_energy_total_setpoint,
		get_params()->tecs.total_energy_kp,
		get_params()->tecs.total_energy_ki,
		1,
		0,
		1,
		get_params()->perf.throttle_cruise,
		_plane->dt_s
	);
}
