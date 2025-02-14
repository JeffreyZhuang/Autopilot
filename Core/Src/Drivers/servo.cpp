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
	// The servo responds for the pulse width of 0.5ms to 2.5ms
	// Low Duty Cycle Percentage = 0.5ms/20ms x 100% = 2.5%
	// High Duty Cycle Percentage = 2.5ms/20ms x 100% = 12.5%

	// Since ARR is 1000
	// Low Duty = 2.5% of 1000 = 25
	// High Duty = 12.5% of 1000 = 125

	// Since servo has angles between 0 to 180 degrees
	// and pulse values between 25 to 125
	// Pulse value for 1 degree of rotation is (125 - 25) / 180
	// Then the pulse value for the capture/compare register is
	// 25 + deg*((125 - 25) / 180)
	float pulse_per_deg = (125.0f - 25.0f) / 180.0f;
	__HAL_TIM_SET_COMPARE(_tim, _channel, 25 + deg * pulse_per_deg);
}

void Servo::set_period(uint16_t us)
{
	// Since 50Hz PWM frequency, the period is 20000us
	// Duty cycle percentage = us / 20000
	// Since ARR is 1000
	// Duty = 1000 * (us / 20000)
	__HAL_TIM_SET_COMPARE(_tim, _channel, 1000 * (us / 20000));
}
