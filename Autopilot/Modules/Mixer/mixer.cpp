#include <Modules/Mixer/mixer.h>

Mixer::Mixer(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

// Convert control setpoints into duty cycle values
// And send the signals to motors
void Mixer::update()
{
	switch (_plane->system_mode)
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
	_hal->set_pwm(0, 0, 0, 0, 0, 0, 0, 0);
}

void Mixer::update_startup()
{
	_hal->set_pwm(0, 0, 0, get_params()->pwm_min_thr, 0, 0, 0, 0);
}

void Mixer::update_flight()
{
	_elevator_duty = map(
		get_params()->pwm_rev_ele ? -_plane->ele_cmd : _plane->ele_cmd,
		 -1,
		 1,
		 get_params()->pwm_min_ele,
		 get_params()->pwm_max_ele);
	_rudder_duty = map(
		get_params()->pwm_rev_rud ? -_plane->rud_cmd : _plane->rud_cmd,
		 -1,
		 1,
		 get_params()->pwm_min_rud,
		 get_params()->pwm_max_rud);
	_throttle_duty = map(
		_plane->thr_cmd,
		 0,
		 1,
		 get_params()->pwm_min_thr,
		 get_params()->pwm_max_thr);
	_hal->set_pwm(_rudder_duty, _elevator_duty, 0, _throttle_duty, 0, 0, 0, 0);
}
