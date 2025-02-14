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

void Flight_hal::set_elevator_duty(uint16_t duty_us)
{
	// deg is from -1 to 1, not degrees, need to change later
	servo_elevator.set_duty(duty_us);
}

void Flight_hal::set_rudder_duty(uint16_t duty_us)
{
	// deg is from -1 to 1, not degrees, need to change later
	servo_aileron.set_duty(duty_us);
}

void Flight_hal::set_throttle_duty(uint16_t duty_us)
{

}
