#include "pid.h"

PID::PID(float kP, float kI, float kD, float integral_limit, float output_min, float output_max)
{
    _kP = kP;
    _kI = kI;
    _kD = kD;
    _integral_limit = integral_limit;
    _output_min = output_min;
    _output_max = output_max;
}

float PID::get_output(float state, float setpoint, float dt)
{
    float error = setpoint - state;

    _integral += error * dt;
    _integral = clamp(_kI * _integral, -_integral_limit, _integral_limit) / _kI;

    float derivative = (error - _prev_error) / dt;
    _prev_error = error;

    float output = _kP * error + _kI * _integral + _kD * derivative;
    output = clamp(output, _output_min, _output_max);

    return output;
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
