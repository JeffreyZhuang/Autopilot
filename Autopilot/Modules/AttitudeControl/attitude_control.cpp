#include <Modules/AttitudeControl/attitude_control.h>

Control::Control(HAL * hal, Plane * plane)
	: roll_controller(false),
	  pitch_controller(false)
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
	_plane->pitch_setpoint = _plane->rc_ele_norm * get_params()->att_ctrl.fbw_ptch_lim;
	_plane->roll_setpoint = _plane->rc_ail_norm * get_params()->att_ctrl.fbw_roll_lim;
	control_roll_ptch();
}

// Manual throttle, hold a pitch angle of TAKEOFF_PTCH and a roll angle of 0
void Control::update_takeoff()
{
	control_roll_ptch_no_integral();
}

void Control::update_mission()
{
	control_roll_ptch();
}

void Control::update_land()
{
	control_roll_ptch();
}

void Control::update_flare()
{
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
		get_params()->att_ctrl.roll_kp,
		get_params()->att_ctrl.roll_ki,
		1,
		-1,
		1,
		0,
		_plane->dt_s
	);
	_plane->ele_cmd = pitch_controller.get_output(
		_plane->ahrs_pitch,
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

void Control::control_roll_ptch_no_integral()
{
	_plane->rud_cmd = roll_controller.get_output(
		_plane->ahrs_roll,
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
		_plane->ahrs_pitch,
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
