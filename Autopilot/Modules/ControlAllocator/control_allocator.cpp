/*
 * control_allocator.cpp
 *
 *  Created on: Feb. 14, 2025
 *      Author: jeffr
 */

#include "control_allocator.h"

Control_allocator::Control_allocator(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Control_allocator::update()
{
	_elevator_duty = map_uint16(_plane->elevator_setpoint, -1, 1, ELEVATOR_MIN_DUTY, ELEVATOR_MAX_DUTY);
	_aileron_duty = map_uint16(_plane->aileron_setpoint, -1, 1, AILERON_MIN_DUTY, AILERON_MAX_DUTY);
	_throttle_duty = map_uint16(_plane->throttle_setpoint, 0, 1, THROTTLE_MIN_DUTY, THROTTLE_MAX_DUTY);

	publish_actuator_controls();
}

void Control_allocator::publish_actuator_controls()
{
	_hal->set_elevator_duty(_elevator_duty);
	_hal->set_rudder_duty(_aileron_duty);
	_hal->set_throttle_duty(_throttle_duty);
}
