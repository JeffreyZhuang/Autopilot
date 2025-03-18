#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_servos()
{
	servo1.init();
	servo2.init();
}

void Flight_hal::set_ail_pwm(uint16_t duty_us)
{
	servo1.set_duty(duty_us);

	hitl_tx_packet.ail_duty = duty_us;
}

void Flight_hal::set_ele_pwm(uint16_t duty_us)
{
	servo2.set_duty(duty_us);

	hitl_tx_packet.ele_duty = duty_us;
}

void Flight_hal::set_rud_pwm(uint16_t duty_us)
{
	hitl_tx_packet.rud_duty = duty_us;
}

void Flight_hal::set_thr_pwm(uint16_t duty_us)
{
	hitl_tx_packet.thr_duty = duty_us;
}
