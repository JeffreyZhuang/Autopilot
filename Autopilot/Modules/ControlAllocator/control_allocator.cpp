#include "control_allocator.h"

Control_allocator::Control_allocator(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Control_allocator::update()
{
	_elevator_duty = map(params.rev_ch[params.elevator_channel] ? -_plane->elevator_setpoint : _plane->elevator_setpoint, -1, 1, ELEVATOR_MIN_DUTY, ELEVATOR_MAX_DUTY);
	_aileron_duty = map(REVERSE_AILERON ? -_plane->aileron_setpoint : _plane->aileron_setpoint, -1, 1, AILERON_MIN_DUTY, AILERON_MAX_DUTY);
	_throttle_duty = map(_plane->throttle_setpoint, 0, 1, THROTTLE_MIN_DUTY, THROTTLE_MAX_DUTY);

	_hal->set_elevator_duty(_elevator_duty);
	_hal->set_rudder_duty(_aileron_duty);
	_hal->set_throttle_duty(_throttle_duty);
}
