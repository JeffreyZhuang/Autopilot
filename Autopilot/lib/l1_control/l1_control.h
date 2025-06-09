#ifndef LIB_L1_CONTROL_L1_CONTROL_H_
#define LIB_L1_CONTROL_L1_CONTROL_H_

// S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
// Proceedings of the AIAA Guidance, Navigation and Control
// Conference, Aug 2004. AIAA-2004-4900.

#include "lib/utils/utils.h"
#include "lib/constants/constants.h"
#include "math.h"

class L1Control
{
public:
	void navigate_waypoints(float x, float y, float vel_x, float vel_y, float ground_speed,
			    			float start_x, float start_y, float end_x, float end_y);
	void navigate_loiter(float x, float y, float vel_x, float vel_y, float ground_speed,
						 float target_x, float target_y, float radius, int8_t direction);

	void set_l1_period(float period) { _l1_period = period; };
	void set_l1_damping(float damping) { _l1_damping = damping; }
	void set_roll_limit(float roll_limit) { _roll_limit = roll_limit; };

	float get_roll_setpoint() { return _roll_setpoint; };
	bool get_circle_mode() { return _circle_mode; };

private:
	// Parameters
	float _l1_period = 0;
	float _l1_damping = 0;
	float _roll_limit = 0;

	// Output
	float _roll_setpoint = 0;
	bool _circle_mode = false;
};

#endif /* LIB_L1_CONTROL_L1_CONTROL_H_ */
