#include <Modules/Mixer/mixer.h>

Control_allocator::Control_allocator(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

// Convert control setpoints into duty cycle values
void Control_allocator::update()
{
	_elevator_duty = map(params.rev_ch[params.elevator_ch] ? -_plane->elevator_setpoint : _plane->elevator_setpoint,
						 -1,
						 1,
						 params.min_duty[params.elevator_ch],
						 params.max_duty[params.elevator_ch]);
	_aileron_duty = map(params.rev_ch[params.aileron_ch] ? -_plane->aileron_setpoint : _plane->aileron_setpoint,
							 -1,
							 1,
							 params.min_duty[params.aileron_ch],
							 params.max_duty[params.aileron_ch]);
	_throttle_duty = map(params.rev_ch[params.throttle_ch] ? -_plane->throttle_setpoint : _plane->throttle_setpoint,
						 0,
						 1,
						 params.min_duty[params.throttle_ch],
						 params.max_duty[params.throttle_ch]);

	_hal->set_duty(0, _aileron_duty);
	_hal->set_duty(1, _elevator_duty);
	_hal->set_duty(2, _throttle_duty);
}
