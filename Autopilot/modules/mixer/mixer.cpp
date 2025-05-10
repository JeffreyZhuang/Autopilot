#include "modules/mixer/mixer.h"

Mixer::Mixer(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _modes_sub(data_bus->modes_node),
	  _ctrl_cmd_sub(data_bus->ctrl_cmd_node),
	  _position_control_sub(data_bus->position_control_node),
	  _hitl_output_pub(data_bus->hitl_output_node)
{
}

// Convert control setpoints into duty cycle values
// And send the signals to motors
void Mixer::update()
{
	_modes_data = _modes_sub.get();
	_ctrl_cmd_data = _ctrl_cmd_sub.get();

	switch (_modes_data.system_mode)
	{
	case System_mode::LOAD_PARAMS:
		update_config();
		break;
	case System_mode::STARTUP:
		update_startup();
		break;
	case System_mode::FLIGHT:
		update_flight();
		break;
	case System_mode::CALIBRATION:
		break;
	}
}

void Mixer::update_config()
{
	_hal->set_pwm(0, 0, 0, 0, 0, 0);
}

void Mixer::update_startup()
{
	int32_t pwm_min_thr;
	param_get(PWM_MIN_THR, &pwm_min_thr);

	_hal->set_pwm(0, 0, pwm_min_thr, 0, 0, 0);
}

void Mixer::update_flight()
{
	int32_t rev_ele, pwm_min_ele, pwm_max_ele,
			rev_rud, pwm_min_rud, pwm_max_rud,
			pwm_min_thr, pwm_max_thr;

	param_get(PWM_REV_ELE, &rev_ele);

	_elevator_duty = map(
		rev_ele ? -_ctrl_cmd_data.ele_cmd : _ctrl_cmd_data.ele_cmd,
		-1,1, param_get_int32(PWM_MIN_ELE), param_get_int32(PWM_MAX_ELE)
	);

	_rudder_duty = map(
		param_get_int32(PWM_REV_RUD) ? -_ctrl_cmd_data.rud_cmd : _ctrl_cmd_data.rud_cmd,
	    -1, 1, param_get_int32(PWM_MIN_RUD), param_get_int32(PWM_MAX_RUD)
	);

	// TODO: Remove reverse for throttle
	_throttle_duty = map(_position_control.throttle_setpoint, 0, 1, param_get_int32(PWM_MIN_THR),
						 param_get_int32(PWM_MAX_THR));

	if (param_get_int32(ENABLE_HITL))
	{
		// Publish HITL commands to data bus, then telem sends through usb
		_hitl_output_pub.publish(HITL_output_data{_elevator_duty, _rudder_duty, _throttle_duty,
												  _hal->get_time_us()});
	}
	else
	{
		_hal->set_pwm(_elevator_duty, _rudder_duty, _throttle_duty, 0, 0, 0);
	}
}
