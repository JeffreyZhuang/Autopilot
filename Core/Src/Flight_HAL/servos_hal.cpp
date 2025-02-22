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

void Flight_hal::set_duty(uint8_t channel, uint16_t duty_us)
{
	switch (channel)
	{
	case 0:
		servo_aileron.set_duty(duty_us);
		break;
	case 1:
		servo_elevator.set_duty(duty_us);
		break;
	case 2:
		break;
	}
}
