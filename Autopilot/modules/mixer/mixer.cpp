#include <modules/mixer/mixer.h>

Mixer::Mixer(HAL* hal)
	: Module(hal),
	  _modes_sub(Data_bus::get_instance().modes_node),
	  _ctrl_cmd_sub(Data_bus::get_instance().ctrl_cmd_node)
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
	_hal->set_pwm(0, 0, get_params()->mixer.pwm_min_thr, 0, 0, 0);
}

void Mixer::update_flight()
{
	_elevator_duty = map(
		get_params()->mixer.pwm_rev_ele ? -_ctrl_cmd_data.ele_cmd : _ctrl_cmd_data.ele_cmd,
		 -1,
		 1,
		 get_params()->mixer.pwm_min_ele,
		 get_params()->mixer.pwm_max_ele);
	_rudder_duty = map(
		get_params()->mixer.pwm_rev_rud ? -_ctrl_cmd_data.rud_cmd : _ctrl_cmd_data.rud_cmd,
		 -1,
		 1,
		 get_params()->mixer.pwm_min_rud,
		 get_params()->mixer.pwm_max_rud);

	// TODO: Remove reverse
	_throttle_duty = map(
		_ctrl_cmd_data.thr_cmd,
		 0,
		 1,
		 get_params()->mixer.pwm_min_thr,
		 get_params()->mixer.pwm_max_thr);
	_hal->set_pwm(_elevator_duty, _rudder_duty, _throttle_duty, 0, 0, 0);
}
