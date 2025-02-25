#include "pid.h"

PID::PID(bool normalize_180, float dt)
{
    _normalize_180 = normalize_180;
    _dt = dt;
}

float PID::get_output(float state,
					  float setpoint,
					  float kP,
					  float kI,
					  float kD,
					  float integral_limit,
					  float output_min,
					  float output_max,
					  float trim)
{
    float error = setpoint - state;
    if (_normalize_180)
    {
    	error = normalize_angle(error);
    }

    if (kI != 0)
    {
    	_integral += error * _dt;
    	_integral = clamp(_integral, -integral_limit / kI, integral_limit / kI);
    }

    float derivative = (error - _prev_error) / _dt;
    _prev_error = error;

    float output = trim + kP * error + kI * _integral + kD * derivative;
    output = clamp(output, output_min, output_max);

    return output;
}

float PID::get_integral()
{
	return _integral;
}

float PID::clamp(float n, float min, float max)
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

float PID::normalize_angle(float angle) {
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
