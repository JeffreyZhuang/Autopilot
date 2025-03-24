#include "modules/attitude_control/attitude_control.h"

Attitude_control::Attitude_control(HAL * hal, Plane * plane) : Module(hal, plane)
{
}

void Attitude_control::update()
{
	_ahrs_data = _plane->get_ahrs_data(_ahrs_handle);

	if (_plane->system_mode == Plane::System_mode::FLIGHT)
	{
		switch (_plane->flight_mode)
		{
		case Plane::Flight_mode::MANUAL:
			handle_manual_mode();
			break;
		case Plane::Flight_mode::AUTO:
			handle_auto_mode();
			break;
		}
	}
}

void Attitude_control::handle_manual_mode()
{
	switch (_plane->manual_mode)
	{
	case Plane::Manual_mode::DIRECT:
		update_direct();
		break;
	case Plane::Manual_mode::STABILIZED:
		update_stabilized();
		break;
	}
}

void Attitude_control::handle_auto_mode()
{
	switch (_plane->auto_mode)
	{
	case Plane::Auto_mode::TAKEOFF:
		update_takeoff();
		break;
	case Plane::Auto_mode::MISSION:
	case Plane::Auto_mode::LAND:
	case Plane::Auto_mode::FLARE:
		update_mission();
		break;
	case Plane::Auto_mode::TOUCHDOWN:
		update_touchdown();
		break;
	}
}

void Attitude_control::update_direct()
{
	_plane->rud_cmd = _plane->rc_ail_norm;
	_plane->ele_cmd = _plane->rc_ele_norm;
}

void Attitude_control::update_stabilized()
{
	_plane->roll_setpoint = _plane->rc_ail_norm * get_params()->att_ctrl.fbw_roll_lim;
	control_roll_ptch();
}

void Attitude_control::update_takeoff()
{
	control_roll_ptch_no_integral();
}

void Attitude_control::update_mission()
{
	control_roll_ptch();
}

void Attitude_control::update_touchdown()
{
	_plane->rud_cmd = 0;
	_plane->ele_cmd = 0;
}

void Attitude_control::control_roll_ptch()
{
	_plane->rud_cmd = roll_controller.get_output(
		_ahrs_data.roll,
		_plane->roll_setpoint,
		get_params()->att_ctrl.roll_kp,
		get_params()->att_ctrl.roll_ki,
		1,
		-1,
		1,
		0,
		_plane->dt_s
	);

	_plane->ele_cmd = pitch_controller.get_output(
		_ahrs_data.pitch,
		_plane->pitch_setpoint,
		get_params()->att_ctrl.ptch_kp,
		get_params()->att_ctrl.ptch_ki,
		1,
		-1,
		1,
		0,
		_plane->dt_s
	);
}

void Attitude_control::control_roll_ptch_no_integral()
{
	_plane->rud_cmd = roll_controller.get_output(
		_ahrs_data.roll,
		_plane->roll_setpoint,
		get_params()->att_ctrl.roll_kp,
		0,
		0,
		-1,
		1,
		0,
		_plane->dt_s
	);

	_plane->ele_cmd = pitch_controller.get_output(
		_ahrs_data.pitch,
		_plane->pitch_setpoint,
		get_params()->att_ctrl.ptch_kp,
		0,
		0,
		-1,
		1,
		0,
		_plane->dt_s
	);
}
