#include "hal.h"

void HAL::read_sensors()
{
	if (_hal_mode == Hal_mode::FLIGHT)
	{
		read_sensors_flight();
	}
	else if (_hal_mode == Hal_mode::HITL)
	{
		read_sensors_hitl();
	}
}

void HAL::usb_print(char* str)
{
	// USB reserved for HITL
	if (_hal_mode == Hal_mode::FLIGHT)
	{
		usb_print_flight(str);
	}
}

void HAL::set_pwm(uint16_t ele_duty, uint16_t rud_duty, uint16_t thr_duty,
			 	  uint16_t aux1_duty, uint16_t aux2_duty, uint16_t aux3_duty)
{
	if (_hal_mode == Hal_mode::FLIGHT)
	{
		set_pwm_flight(ele_duty, rud_duty, thr_duty,
					   aux1_duty, aux2_duty, aux3_duty);
	}
	else if (_hal_mode == Hal_mode::HITL)
	{
		set_pwm_hitl(ele_duty, rud_duty, thr_duty,
					 aux1_duty, aux2_duty, aux3_duty);
	}
}

void HAL::enable_hitl() noexcept
{
	_hal_mode = Hal_mode::HITL;
}
