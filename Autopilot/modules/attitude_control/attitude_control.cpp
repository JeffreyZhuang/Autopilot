#include <modules/attitude_control/attitude_control.h>

Attitude_control::Attitude_control(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _ahrs_sub(data_bus->ahrs_node),
	  _modes_sub(data_bus->modes_node),
	  _position_control_sub(data_bus->position_control_node),
	  _rc_sub(data_bus->rc_node),
	  _time_sub(data_bus->time_node),
	  _ctrl_cmd_pub(data_bus->ctrl_cmd_node)
{
}

void Attitude_control::update()
{
	_time = _time_sub.get();
	_ahrs_data = _ahrs_sub.get();
	_modes_data = _modes_sub.get();
	_position_control = _position_control_sub.get();
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

void Attitude_control::control_roll_ptch()
{
	_ctrl_cmd_data.rud_cmd = roll_controller.get_output(
		_ahrs_data.roll,
		_position_control.roll_setpoint,
		ATT_ROLL_KP.get(),
		ATT_ROLL_KI.get(),
		1,
		-1,
		1,
		0,
		_time.dt_s
	);

	_ctrl_cmd_data.ele_cmd = pitch_controller.get_output(
		_ahrs_data.pitch,
		_position_control.pitch_setpoint,
		ATT_PTCH_KP.get(),
		ATT_PTCH_KI.get(),
		1,
		-1,
		1,
		0,
		_time.dt_s
	);
}

void Attitude_control::control_roll_ptch_no_integral()
{
	_ctrl_cmd_data.rud_cmd = roll_controller.get_output(
		_ahrs_data.roll,
		_position_control.roll_setpoint,
		ATT_ROLL_KP.get(),
		0,
		0,
		-1,
		1,
		0,
		_time.dt_s
	);

	_ctrl_cmd_data.ele_cmd = pitch_controller.get_output(
		_ahrs_data.pitch,
		_position_control.pitch_setpoint,
		ATT_PTCH_KP.get(),
		0,
		0,
		-1,
		1,
		0,
		_time.dt_s
	);
}
