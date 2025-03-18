#include "PITL_HAL/pitl_hal.h"

// PITL_HAL uses a duty cycle from 1000 to 2000 us
// You need to configure parameters to match this
void Pitl_hal::set_ail_pwm(uint16_t duty_us)
{
	pitl_tx_packet.ail = map(duty_us, 1000, 2000, -1, 1);
}

void Pitl_hal::set_ele_pwm(uint16_t duty_us)
{
	pitl_tx_packet.ele = map(duty_us, 1000, 2000, -1, 1);
}

void Pitl_hal::set_rud_pwm(uint16_t duty_us)
{
	pitl_tx_packet.rud = map(duty_us, 1000, 2000, -1, 1);
}

void Pitl_hal::set_thr_pwm(uint16_t duty_us)
{
	pitl_tx_packet.thr = map(duty_us, 1000, 2000, 0, 1);
}
