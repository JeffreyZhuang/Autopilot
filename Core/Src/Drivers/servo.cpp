#include <servo.h>

Servo::Servo(TIM_HandleTypeDef* tim, uint32_t channel)
{
	_tim = tim;
	_channel = channel;
}

void Servo::init()
{
	// Note: ARR must be set to 1000
	HAL_TIM_PWM_Start(_tim, _channel);

	set_angle(90); // Set to mid point
}

void Servo::set_angle(uint8_t deg)
{
	if (deg > 180)
	{
		deg = 180;
	}

	// Since 50Hz PWM frequency, the period is 20ms
	// The servo responds for the pulse width of 1ms to 2ms
	// Low Duty Cycle Percentage = 1ms/20ms x 100% = 5%
	// High Duty Cycle Percentage = 2ms/20ms x 100% = 10%

	// Since ARR is 1000
	// Low Duty = 5% of 1000 = 50
	// High Duty = 10% of 1000 = 100

	// Since servo has angles between 0 to 180 degrees
	// Pulse value for 1 degree of rotation is (100 - 50) / 180
	// Then the pulse value for the capture/compare register is
	// 50 + deg*((100 - 50) / 180)
	__HAL_TIM_SET_COMPARE(_tim, _channel, 50 + deg * (100 - 50) / 180);
}
