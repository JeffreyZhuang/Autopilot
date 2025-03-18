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
	_hal->set_thr_pwm(0);
}

void Mixer::update_startup()
{
	_hal->set_thr_pwm(get_params()->pwm_min_thr);
}

void Mixer::update_flight()
{
	_elevator_duty = map(
		get_params()->pwm_rev_ele ? -_plane->elevator_setpoint : _plane->elevator_setpoint,
		 -1,
		 1,
		 get_params()->pwm_min_ele,
		 get_params()->pwm_max_ele);
	_aileron_duty = map(
		get_params()->pwm_rev_ail ? -_plane->aileron_setpoint : _plane->aileron_setpoint,
		 -1,
		 1,
		 get_params()->pwm_min_ail,
		 get_params()->pwm_max_ail);
	_throttle_duty = map(
		_plane->throttle_setpoint,
		 0,
		 1,
		 get_params()->pwm_min_thr,
		 get_params()->pwm_max_thr);

	_hal->set_ail_pwm(_aileron_duty);
	_hal->set_ele_pwm(_elevator_duty);
	_hal->set_thr_pwm(_throttle_duty);
}
