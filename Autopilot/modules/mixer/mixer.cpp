#include "modules/mixer/mixer.h"

Mixer::Mixer(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _modes_sub(data_bus->modes_node),
	  _ctrl_cmd_sub(data_bus->ctrl_cmd_node),
	  _tecs_sub(data_bus->tecs_node),
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
	case System_mode::CONFIG:
		update_config();
		break;
	case System_mode::STARTUP:
		update_startup();
		break;
	case System_mode::FLIGHT:
		update_flight();
		break;
	}
}

void Mixer::update_config()
{
	_hal->set_pwm(0, 0, 0, 0, 0, 0);
}

void Mixer::update_startup()
{
	int32_t pwm_min_thr = 0;
	param_get_int32(param_find(PWM_MIN_THR), &pwm_min_thr);

	_hal->set_pwm(0, 0, pwm_min_thr, 0, 0, 0);
}

void Mixer::update_flight()
{
	int32_t pwm_rev_ele, pwm_rev_rud, pwm_rev_thr,
			pwm_min_ele, pwm_min_rud, pwm_min_thr,
			pwm_max_ele, pwm_max_rud, pwm_max_thr;
	param_get_int32(param_find(PWM_REV_ELE), &pwm_rev_ele);
	param_get_int32(param_find(PWM_REV_RUD), &pwm_rev_rud);
	param_get_int32(param_find(PWM_REV_THR), &pwm_rev_thr);
	param_get_int32(param_find(PWM_MIN_ELE), &pwm_min_ele);
	param_get_int32(param_find(PWM_MIN_RUD), &pwm_min_rud);
	param_get_int32(param_find(PWM_MIN_THR), &pwm_min_thr);
	param_get_int32(param_find(PWM_MAX_ELE), &pwm_max_ele);
	param_get_int32(param_find(PWM_MAX_RUD), &pwm_max_rud);
	param_get_int32(param_find(PWM_MAX_THR), &pwm_max_thr);

	_elevator_duty = map(pwm_rev_ele ? -_ctrl_cmd_data.ele_cmd : _ctrl_cmd_data.ele_cmd,
						 -1,1, pwm_min_ele, pwm_max_ele);
	_rudder_duty = map(pwm_rev_rud ? -_ctrl_cmd_data.rud_cmd : _ctrl_cmd_data.rud_cmd,
					   -1, 1, pwm_min_rud, pwm_max_rud);

	// TODO: Remove reverse for throttle
	_throttle_duty = map(_tecs_data.thr_cmd, 0, 1, pwm_min_thr, pwm_max_thr);

	if (get_params()->hitl.enable)
	{
		// Publish HITL commands to data bus, then telem sends through usb
		_hitl_output_pub.publish(HITL_output_data{_elevator_duty, _rudder_duty, _throttle_duty, _hal->get_time_us()});
	}
	else
	{
		_hal->set_pwm(_elevator_duty, _rudder_duty, _throttle_duty, 0, 0, 0);
	}
}
