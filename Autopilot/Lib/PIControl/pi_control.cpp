#include "Lib/PIControl/pi_control.h"

PI_control::PI_control(bool normalize_180)
{
    _normalize_180 = normalize_180;
}

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
    if (_normalize_180)
    {
    	error = normalize_angle(error);
    }

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

float PI_control::normalize_angle(float angle) {
    while (angle >= 180.0)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0)
    {
        angle += 360.0f;
    }

    return angle;
}
