/*
 * servos_hal.cpp
 *
 *  Created on: Dec 29, 2024
 *      Author: jeffr
 */

#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_servos()
{
	servo_elevator.init();
	servo_aileron.init();
}

void Flight_hal::set_elevator(float deg)
{
	// deg is from -1 to 1, not degrees, need to change later
	servo_elevator.set_angle((uint8_t)(90 + deg * 90));
}

void Flight_hal::set_rudder(float deg)
{
	// deg is from -1 to 1, not degrees, need to change later
	servo_aileron.set_angle((uint8_t)(90 + deg * 90));
}

void Flight_hal::set_throttle(float throttle)
{

}
