#include "attitude_control.h"

AttitudeControl::AttitudeControl(HAL* hal, DataBus* data_bus)
	: Module(hal, data_bus),
	  _ahrs_sub(data_bus->ahrs_node),
	  _modes_sub(data_bus->modes_node),
	  _position_control_sub(data_bus->position_control_node),
	  _rc_sub(data_bus->rc_node),
	  _ctrl_cmd_pub(data_bus->ctrl_cmd_node)
{
}

void AttitudeControl::update_time()
{
	const uint64_t time = _hal->get_time_us();
	_dt = clamp((time - _last_time) * US_TO_S, DT_MIN, DT_MAX);
	_last_time = time;
}

void AttitudeControl::update_parameters()
{
	param_get(ATT_ROLL_KP, &_roll_kp);
	param_get(ATT_ROLL_KI, &_roll_ki);
	param_get(ATT_PTCH_KP, &_ptch_kp);
	param_get(ATT_PTCH_KI, &_ptch_ki);
}

void AttitudeControl::poll_vehicle_data()
{
	_ahrs_data = _ahrs_sub.get();
	_modes_data = _modes_sub.get();
	_position_control = _position_control_sub.get();
	_rc_data = _rc_sub.get();
}

void AttitudeControl::update()
{
	update_time();
	update_parameters();
	poll_vehicle_data();

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

	publish_status();
}

void AttitudeControl::handle_manual_mode()
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

void AttitudeControl::update_direct()
{
	_ctrl_cmd_data.rud_cmd = _rc_data.ail_norm;
	_ctrl_cmd_data.ele_cmd = _rc_data.ele_norm;
}

void AttitudeControl::update_stabilized()
{
	control_roll_ptch();
}

void AttitudeControl::handle_auto_mode()
{
	if (_modes_data.auto_mode == Auto_mode::DETECT)
	{
		// Reset integral here
	}

	control_roll_ptch();
}

void AttitudeControl::publish_status()
{
	_ctrl_cmd_data.timestamp = _hal->get_time_us();
	_ctrl_cmd_pub.publish(_ctrl_cmd_data);
}

void AttitudeControl::control_roll_ptch()
{
	_ctrl_cmd_data.rud_cmd = roll_controller.get_output(
		_ahrs_data.roll, _position_control.roll_setpoint,
		_roll_kp, _roll_ki, 1, -1, 1, 0, _dt
	);

	_ctrl_cmd_data.ele_cmd = pitch_controller.get_output(
		_ahrs_data.pitch, _position_control.pitch_setpoint,
		_ptch_kp, _ptch_ki, 1, -1, 1, 0, _dt
	);
}
