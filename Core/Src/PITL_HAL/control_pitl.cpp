#include "PITL_HAL/pitl_hal.h"

// PITL uses aileron for channel 0, elevator for channel 1, and throttle for channel 2
// and a duty cycle from 1000 to 2000 us
// You need to configure parameters to match this
void Pitl_hal::set_duty(uint8_t channel, uint16_t duty_us)
{
	switch (channel)
	{
	case 0:
		pitl_tx_packet.aileron = map(duty_us, 1000, 2000, -1, 1);
		break;
	case 1:
		pitl_tx_packet.elevator = map(duty_us, 1000, 2000, -1, 1);
		break;
	case 2:
		pitl_tx_packet.throttle = map(duty_us, 1000, 2000, 0, 1);
		break;
	}
}
