#include "lib/pi_control/pi_control.h"

float PI_control::get_output(float state,
					  float setpoint,
					  float kP,
					  float kI,
					  float integral_limit,
					  float output_min,
					  float output_max,
					  float trim,
					  float dt)
{
    float error = setpoint - state;

    if (kI != 0)
    {
    	_integral += error * dt;
    	_integral = clamp(_integral, -integral_limit / kI, integral_limit / kI);
    }

    float output = trim + kP * error + kI * _integral;
    output = clamp(output, output_min, output_max);

    return output;
}

float PI_control::get_integral()
{
	return _integral;
}

float PI_control::clamp(float n, float min, float max)
{
    if (n > max)
    {
        n = max;
    }

    if (n < min)
    {
        n = min;
    }

    return n;
}
