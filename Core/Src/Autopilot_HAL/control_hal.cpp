#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::init_servos()
{
	servo1.init();
	servo2.init();
}

void AutopilotHAL::set_pwm(uint16_t ele_duty, uint16_t rud_duty, uint16_t thr_duty,
		  	  	  	  	 uint16_t aux1_duty, uint16_t aux2_duty, uint16_t aux3_duty)
{
	servo1.set_duty(ele_duty);
	servo2.set_duty(rud_duty);
}
