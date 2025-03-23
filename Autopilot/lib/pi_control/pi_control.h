#ifndef PID_H_
#define PID_H_

class PI_control
{
public:
    float get_output(float state,
    				 float setpoint,
					 float kP,
        			 float kI,
    				 float integral_limit,
    				 float output_min,
    				 float output_max,
    				 float trim,
					 float dt);
    float get_integral();

private:
    float _integral = 0;
    float clamp(float n, float min, float max);
};

#endif /* PID_H_ */
