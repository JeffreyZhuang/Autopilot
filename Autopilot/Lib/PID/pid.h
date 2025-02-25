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
	// normalize_180 is special for heading PID where it finds nearest path by normalizing angle to [-180, 180) degrees
    PID(bool normalize_180, float dt);

    float get_output(float state,
    				 float setpoint,
					 float kP,
        			 float kI,
    				 float kD,
    				 float integral_limit,
    				 float output_min,
    				 float output_max,
    				 float trim);
    float get_integral();
private:
    float clamp(float n, float min, float max);
    float normalize_angle(float angle);

    float _integral = 0;
    float _prev_error = 0;
    bool _normalize_180;
    float _dt;
};

#endif /* PID_H_ */
