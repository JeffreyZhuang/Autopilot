#include "pid.h"

PID::PID(float kP, float kI, float kD, float integral_limit, float output_min, float output_max, bool normalize_180)
{
    _kP = kP;
    _kI = kI;
    _kD = kD;
    _integral_limit = integral_limit; // Limit for entire integral term
    _output_min = output_min;
    _output_max = output_max;
    _normalize_180 = normalize_180;
}

float PID::get_output(float state, float setpoint, float dt)
{
    float error = setpoint - state;
    if (_normalize_180)
    {
    	error = normalize_angle(error);
    }

    if (_kI != 0)
    {
    	_integral += error * dt;
    	_integral = clamp(_integral, -_integral_limit / _kI, _integral_limit / _kI);
    }

    float derivative = (error - _prev_error) / dt;
    _prev_error = error;

    float output = _kP * error + _kI * _integral + _kD * derivative;
    output = clamp(output, _output_min, _output_max);

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
    while (angle >= 180.0) {
        angle -= 360.0f;
    }
    while (angle < -180.0) {
        angle += 360.0f;
    }
    return angle;
}
