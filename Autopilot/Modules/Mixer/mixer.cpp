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
	_hal->set_duty(0, 0);
	_hal->set_duty(1, 0);
	_hal->set_duty(2, 0);
}

void Mixer::update_startup()
{
	_hal->set_duty(get_params()->aileron_ch, 0);
	_hal->set_duty(get_params()->elevator_ch, 0);
	_hal->set_duty(get_params()->throttle_ch, get_params()->min_duty[get_params()->throttle_ch]);
}

void Mixer::update_flight()
{
	_elevator_duty = map(get_params()->rev_ch[get_params()->elevator_ch] ? -_plane->elevator_setpoint : _plane->elevator_setpoint,
						 -1,
						 1,
						 get_params()->min_duty[get_params()->elevator_ch],
						 get_params()->max_duty[get_params()->elevator_ch]);
	_aileron_duty = map(get_params()->rev_ch[get_params()->aileron_ch] ? -_plane->aileron_setpoint : _plane->aileron_setpoint,
							 -1,
							 1,
							 get_params()->min_duty[get_params()->aileron_ch],
							 get_params()->max_duty[get_params()->aileron_ch]);
	_throttle_duty = map(get_params()->rev_ch[get_params()->throttle_ch] ? -_plane->throttle_setpoint : _plane->throttle_setpoint,
						 0,
						 1,
						 get_params()->min_duty[get_params()->throttle_ch],
						 get_params()->max_duty[get_params()->throttle_ch]);

	_hal->set_duty(0, _aileron_duty);
	_hal->set_duty(1, _elevator_duty);
	_hal->set_duty(2, _throttle_duty);
}
