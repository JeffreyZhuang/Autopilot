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
    PID(float kP,
    	float kI,
		float kD,
		float integral_limit,
		float output_min,
		float output_max,
		float trim,
		bool normalize_180);

    float get_output(float state, float setpoint, float dt);
    float get_integral();
private:
    float clamp(float n, float min, float max);
    float normalize_angle(float angle);

    float _kP;
    float _kI;
    float _kD;
    float _integral_limit;
    float _integral = 0;
    float _prev_error = 0;
    float _output_max;
    float _output_min;
    float _trim;
    bool _normalize_180;
};

#endif /* PID_H_ */
