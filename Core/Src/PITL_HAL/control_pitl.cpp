#include "pitl_hal.h"

void Pitl_hal::set_duty(uint8_t channel, uint16_t duty_us)
{
	pitl_tx_packet.value[channel] = map(duty_us, params.min_duty[channel], params.max_duty[channel], -1, 1);
}
