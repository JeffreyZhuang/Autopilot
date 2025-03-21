#include "control.h"

Control::Control(HAL * hal, Plane * plane)
	: roll_controller(false),
	  pitch_controller(false),
	  hdg_controller(true),
	  alt_controller(false),
	  speed_controller(false)
{
	_hal = hal;
	_plane = plane;
}

void Control::update()
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

void Control::handle_manual_mode()
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

void Control::handle_auto_mode()
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

// Read from radio and send commands directly to servos
void Control::update_direct()
{
	_plane->rud_cmd = _plane->rc_ail_norm;
	_plane->ele_cmd = _plane->rc_ele_norm;
	_plane->thr_cmd = _plane->rc_thr_norm;
}

// Pilot commands roll and pitch angles, throttle is manual
void Control::update_stabilized()
{
	_plane->thr_cmd = _plane->rc_thr_norm;
	_plane->pitch_setpoint = _plane->rc_ele_norm * get_params()->stab_ptch_lim;
	_plane->roll_setpoint = _plane->rc_ail_norm * get_params()->stab_roll_lim;
	control_roll_ptch();
}

// Manual throttle, hold a pitch angle of TAKEOFF_PTCH and a roll angle of 0
void Control::update_takeoff()
{
	// No integral
	_plane->roll_setpoint = hdg_controller.get_output(
		_plane->ahrs_yaw,
		_plane->guidance_hdg_setpoint,
		get_params()->hdg_kp,
		0,
		0,
		-get_params()->takeoff_roll_lim,
		get_params()->takeoff_roll_lim,
		0,
		_plane->dt_s
	);
	_plane->pitch_setpoint = get_params()->takeoff_ptch;

	// No integral
	_plane->rud_cmd = roll_controller.get_output(
		_plane->ahrs_roll,
		_plane->roll_setpoint,
		get_params()->roll_kp,
		0,
		0,
		-1,
		1,
		0,
		_plane->dt_s
	);
	_plane->ele_cmd = pitch_controller.get_output(
		_plane->ahrs_pitch,
		_plane->pitch_setpoint,
		get_params()->ptch_kp,
		0,
		0,
		-1,
		1,
		0,
		_plane->dt_s
	);
	_plane->thr_cmd = _plane->rc_thr_norm;
}

// Track guidance altitude and heading setpoints at a speed of AIRSPEED_CUIRSE
void Control::update_mission()
{
	control_alt_spd_hdg();
	control_roll_ptch();
}

// Track approach guidance altitude and heading setpoints at the reduced speed of AIRSPEED_LANDING
void Control::update_land()
{
	control_alt_spd_hdg();
	control_roll_ptch();
}

// Cut throttle, set roll to 0 and track flare guidance altitude setpoints
void Control::update_flare()
{
	_plane->pitch_setpoint = alt_controller.get_output(
	    _plane->tecs_energy_diff,
	    _plane->tecs_energy_diff_setpoint,
	    get_params()->alt_kp,
	    get_params()->alt_ki,
		get_params()->ptch_lim_deg,
	    -get_params()->ptch_lim_deg,
	    get_params()->ptch_lim_deg,
	    0,
		_plane->dt_s
	);
	_plane->roll_setpoint = 0;
	_plane->thr_cmd = 0;
	control_roll_ptch();
}

void Control::update_touchdown()
{
	_plane->rud_cmd = 0;
	_plane->ele_cmd = 0;
	_plane->thr_cmd = 0;
}

void Control::control_roll_ptch()
{
	_plane->rud_cmd = roll_controller.get_output(
		_plane->ahrs_roll,
		_plane->roll_setpoint,
		get_params()->roll_kp,
		get_params()->roll_ki,
		1,
		-1,
		1,
		0,
		_plane->dt_s
	);
	_plane->ele_cmd = pitch_controller.get_output(
		_plane->ahrs_pitch,
		_plane->pitch_setpoint,
		get_params()->ptch_kp,
		get_params()->ptch_ki,
		1,
		-1,
		1,
		0,
		_plane->dt_s
	);
}

void Control::control_alt_spd_hdg()
{
	_plane->pitch_setpoint = alt_controller.get_output(
		_plane->tecs_energy_diff,
		_plane->tecs_energy_diff_setpoint,
		get_params()->alt_kp,
		get_params()->alt_ki,
		get_params()->ptch_lim_deg,
		-get_params()->ptch_lim_deg,
		get_params()->ptch_lim_deg,
		0,
		_plane->dt_s
	);
	_plane->thr_cmd = speed_controller.get_output(
		_plane->tecs_energy_total,
		_plane->tecs_energy_total_setpoint,
		get_params()->thr_kp,
		get_params()->thr_ki,
		1,
		0,
		1,
		get_params()->throttle_cruise,
		_plane->dt_s
	);
	_plane->roll_setpoint = hdg_controller.get_output(
		_plane->ahrs_yaw,
		_plane->guidance_hdg_setpoint,
		get_params()->hdg_kp,
		get_params()->hdg_ki,
		get_params()->roll_lim_deg,
		-get_params()->roll_lim_deg,
		get_params()->roll_lim_deg,
		0,
		_plane->dt_s
	);
}
