#include "modules/attitude_control/attitude_control.h"

Attitude_control::Attitude_control(HAL* hal)
	: Module(hal),
	  _ahrs_sub(Data_bus::get_instance().ahrs_data),
	  _tecs_sub(Data_bus::get_instance().tecs_data),
	  _modes_sub(Data_bus::get_instance().modes_data),
	  _l1_sub(Data_bus::get_instance().l1_data),
	  _rc_sub(Data_bus::get_instance().rc_data),
	  _time_sub(Data_bus::get_instance().time_data),
	  _ctrl_cmd_pub(Data_bus::get_instance().ctrl_cmd_data)
{
}

void Attitude_control::update()
{
	_time_data = _time_sub.get();
	_ahrs_data = _ahrs_sub.get();
	_modes_data = _modes_sub.get();
	_l1_data = _l1_sub.get();
	_tecs_data = _tecs_sub.get();
	_rc_data = _rc_sub.get();

	if (_modes_data.system_mode == System_mode::FLIGHT)
	{
		switch (_modes_data.flight_mode)
		{
		case Flight_mode::MANUAL:
			handle_manual_mode();
			break;
		case Flight_mode::AUTO:
			handle_auto_mode();
			break;
		}
	}

	_ctrl_cmd_pub.publish(_ctrl_cmd_data);
}

void Attitude_control::handle_manual_mode()
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

void Attitude_control::handle_auto_mode()
{
	switch (_modes_data.auto_mode)
	{
	case Auto_mode::TAKEOFF:
		update_takeoff();
		break;
	case Auto_mode::MISSION:
	case Auto_mode::LAND:
	case Auto_mode::FLARE:
		update_mission();
		break;
	case Auto_mode::TOUCHDOWN:
		update_touchdown();
		break;
	}
}

void Attitude_control::update_direct()
{
	_ctrl_cmd_data.rud_cmd = _rc_data.ail_norm;
	_ctrl_cmd_data.ele_cmd = _rc_data.ele_norm;
}

void Attitude_control::update_stabilized()
{
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
	_ctrl_cmd_data.rud_cmd = 0;
	_ctrl_cmd_data.ele_cmd = 0;
}

void Attitude_control::control_roll_ptch()
{
	_ctrl_cmd_data.rud_cmd = roll_controller.get_output(
		_ahrs_data.roll,
		_l1_data.roll_setpoint,
		get_params()->att_ctrl.roll_kp,
		get_params()->att_ctrl.roll_ki,
		1,
		-1,
		1,
		0,
		_time_data.dt_s
	);

	_ctrl_cmd_data.ele_cmd = pitch_controller.get_output(
		_ahrs_data.pitch,
		_tecs_data.pitch_setpoint,
		get_params()->att_ctrl.ptch_kp,
		get_params()->att_ctrl.ptch_ki,
		1,
		-1,
		1,
		0,
		_time_data.dt_s
	);
}

void Attitude_control::control_roll_ptch_no_integral()
{
	_ctrl_cmd_data.rud_cmd = roll_controller.get_output(
		_ahrs_data.roll,
		_l1_data.roll_setpoint,
		get_params()->att_ctrl.roll_kp,
		0,
		0,
		-1,
		1,
		0,
		_time_data.dt_s
	);

	_ctrl_cmd_data.ele_cmd = pitch_controller.get_output(
		_ahrs_data.pitch,
		_tecs_data.pitch_setpoint,
		get_params()->att_ctrl.ptch_kp,
		0,
		0,
		-1,
		1,
		0,
		_time_data.dt_s
	);
}
