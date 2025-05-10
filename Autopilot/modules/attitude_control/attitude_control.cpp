#include "attitude_control.h"

AttitudeControl::AttitudeControl(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _ahrs_sub(data_bus->ahrs_node),
	  _modes_sub(data_bus->modes_node),
	  _position_control_sub(data_bus->position_control_node),
	  _rc_sub(data_bus->rc_node),
	  _ctrl_cmd_pub(data_bus->ctrl_cmd_node)
{
}

void AttitudeControl::update()
{
	const uint64_t time = _hal->get_time_us();
	_dt = clamp((time - _last_time) * US_TO_S, DT_MIN, DT_MAX);
	_last_time = time;

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

void AttitudeControl::handle_auto_mode()
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

void AttitudeControl::update_direct()
{
	_ctrl_cmd_data.rud_cmd = _rc_data.ail_norm;
	_ctrl_cmd_data.ele_cmd = _rc_data.ele_norm;
}

void AttitudeControl::update_stabilized()
{
	control_roll_ptch();
}

void AttitudeControl::update_takeoff()
{
	control_roll_ptch_no_integral();
}

void AttitudeControl::update_mission()
{
	control_roll_ptch();
}

void AttitudeControl::control_roll_ptch()
{
	float roll_kp, roll_ki, ptch_kp, ptch_ki;

	// Get roll and pitch P and I gains
	param_get(ATT_ROLL_KP, &roll_kp);
	param_get(ATT_ROLL_KI, &roll_ki);
	param_get(ATT_PTCH_KP, &ptch_kp);
	param_get(ATT_PTCH_KI, &ptch_ki);

	_ctrl_cmd_data.rud_cmd = roll_controller.get_output(
		_ahrs_data.roll, _position_control.roll_setpoint,
		roll_kp, roll_ki, 1, -1, 1, 0, _dt
	);

	_ctrl_cmd_data.ele_cmd = pitch_controller.get_output(
		_ahrs_data.pitch, _position_control.pitch_setpoint,
		ptch_kp, ptch_ki, 1, -1, 1, 0, _dt
	);
}

// Use update parameters function like PX4

void AttitudeControl::control_roll_ptch_no_integral()
{
	float roll_kp, ptch_kp;

	// Get roll and pitch P gains
	param_get(ATT_ROLL_KP, &roll_kp);
	param_get(ATT_PTCH_KP, &ptch_kp);

	_ctrl_cmd_data.rud_cmd = roll_controller.get_output(
		_ahrs_data.roll, _position_control.roll_setpoint,
		roll_kp, 0, 0, -1, 1, 0, _dt
	);

	_ctrl_cmd_data.ele_cmd = pitch_controller.get_output(
		_ahrs_data.pitch, _position_control.pitch_setpoint,
		ptch_kp, 0, 0, -1, 1, 0, _dt
	);
}
