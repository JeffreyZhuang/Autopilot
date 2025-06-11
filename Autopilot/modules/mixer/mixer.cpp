#include "modules/mixer/mixer.h"

Mixer::Mixer(HAL* hal, DataBus* data_bus)
	: Module(hal, data_bus),
	  _modes_sub(data_bus->modes_node),
	  _ctrl_cmd_sub(data_bus->ctrl_cmd_node),
	  _position_control_sub(data_bus->position_control_node),
	  _hitl_output_pub(data_bus->hitl_output_node)
{
}

void Mixer::parameters_update()
{
	param_get(PWM_MIN_ELE, &_pwm_min_ele);
	param_get(PWM_MAX_ELE, &_pwm_max_ele);
	param_get(PWM_MIN_RUD, &_pwm_min_rud);
	param_get(PWM_MAX_RUD, &_pwm_max_rud);
	param_get(PWM_MIN_THR, &_pwm_min_thr);
	param_get(PWM_MAX_THR, &_pwm_max_thr);
	param_get(PWM_REV_ELE, &_rev_ele);
	param_get(PWM_REV_RUD, &_rev_rud);
}

void Mixer::update()
{
	parameters_update();

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
	_hal->set_pwm(0, 0, _pwm_min_thr, 0, 0, 0);
}

void Mixer::update_flight()
{
	_elevator_duty = map(
		_rev_ele ? -_ctrl_cmd_data.ele_cmd : _ctrl_cmd_data.ele_cmd,
		-1, 1, _pwm_min_ele, _pwm_max_ele
	);

	_rudder_duty = map(
		_rev_rud ? -_ctrl_cmd_data.rud_cmd : _ctrl_cmd_data.rud_cmd,
	    -1, 1, _pwm_min_rud, _pwm_max_rud
	);

	_throttle_duty = map(_position_control.throttle_setpoint, 0, 1, _pwm_min_thr, _pwm_max_thr);

	_hitl_output_pub.publish(HITL_output_data{
		_elevator_duty,
		_rudder_duty,
		_throttle_duty,
		_hal->get_time_us()
	});

	_hal->set_pwm(_elevator_duty, _rudder_duty, _throttle_duty, 0, 0, 0);
}
