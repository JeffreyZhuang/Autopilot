/*
 * pid.h
 *
 *  Created on: Dec. 29, 2024
 *      Author: jeffr
 */

#ifndef PID_H_
#define PID_H_

class PID
{
public:
    PID(float kP, float kI, float kD, float integral_limit);
    float get_output(float state, float setpoint, float dt);
private:
    float clamp(float n, float min, float max);
    float _kP;
    float _kI;
    float _kD;
    float _integral_limit;
    float _integral = 0;
    float _prev_error = 0;
};

#endif /* PID_H_ */
