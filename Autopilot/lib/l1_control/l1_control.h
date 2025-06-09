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
	void navigate_loiter();
	void navigate_heading();

	void set_l1_period(float period) { _l1_period = period; };
	void set_roll_limit(float roll_limit) { _roll_limit = roll_limit; };

	float get_roll_setpoint() { return _roll_setpoint; };

private:
	// Local position in NED frame
	struct Position {
		float x;
		float y;
	};

	// Parameters
	float _l1_period = 0;
	float _roll_limit = 0;

	// Output
	float _roll_setpoint = 0;
};

#endif /* LIB_L1_CONTROL_L1_CONTROL_H_ */
