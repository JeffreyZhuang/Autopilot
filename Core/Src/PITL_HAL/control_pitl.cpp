#include "pitl_hal.h"

void Pitl_hal::set_duty(uint8_t channel, uint16_t duty_us)
{
	if (channel == params.aileron_ch)
	{
		pitl_tx_packet.aileron = map(duty_us, params.min_duty[channel], params.max_duty[channel], -1, 1);
	}
	else if (channel == params.elevator_ch)
	{
		pitl_tx_packet.elevator = map(duty_us, params.min_duty[channel], params.max_duty[channel], -1, 1);
	}
	else if (channel == params.throttle_ch)
	{
		pitl_tx_packet.throttle = map(duty_us, params.min_duty[channel], params.max_duty[channel], 0, 1);
	}
}
