/*
 * servos_hal.cpp
 *
 *  Created on: Dec 29, 2024
 *      Author: jeffr
 */

#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_servos()
{
	servo1.init();
	servo2.init();
}

void Flight_hal::set_duty(uint8_t channel, uint16_t duty_us)
{
	switch (channel)
	{
	case 0:
		servo1.set_duty(duty_us);
		break;
	case 1:
		servo2.set_duty(duty_us);
		break;
	case 2:
		break;
	}
}
