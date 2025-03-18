#ifndef PID_H_
#define PID_H_

class PI_control
{
public:
	// normalize_180 is special for heading PID where it finds nearest path by normalizing angle to [-180, 180) degrees
    PI_control(bool normalize_180, float dt);
    float get_output(float state,
    				 float setpoint,
					 float kP,
        			 float kI,
    				 float integral_limit,
    				 float output_min,
    				 float output_max,
    				 float trim);
    float get_integral();
private:
    float clamp(float n, float min, float max);
    float normalize_angle(float angle);

    float _integral = 0;
    bool _normalize_180;
    float _dt;
};

#endif /* PID_H_ */
