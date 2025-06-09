#include "l1_control.h"

void L1Control::navigate_waypoints(float x, float y, float vel_x, float vel_y, float ground_speed,
					   	   	   	   float start_x, float start_y, float end_x, float end_y)
{
	// Calculate track heading (bearing from previous to target waypoint)
	const float trk_hdg = atan2f(end_y - start_y, end_x - start_x);

	// Compute cross-track error (perpendicular distance from aircraft to path)
	const float xte = cosf(trk_hdg) * (y - end_y) - sinf(trk_hdg) * (x - end_x);

	// Calculate L1 distance and scale with speed
	const float l1_dist = fmaxf(_l1_period * ground_speed / M_PI, 1.0);

	// Calculate correction angle
	const float correction_angle = asinf(clamp(xte / l1_dist, -1, 1)); // Domain of acos is [-1, 1]

	// Apply correction angle to track heading to compute heading setpoint
	const float hdg_setpoint = trk_hdg - correction_angle;

	// Calculate plane velocity heading
	const float plane_hdg = atan2f(vel_x, vel_y);

	// Calculate plane heading error
	const float hdg_err = hdg_setpoint - plane_hdg;

	// Calculate lateral acceleration using l1 guidance
	const float lateral_accel = 2 * powf(ground_speed, 2) / l1_dist * sinf(hdg_err);

	// Calculate roll to get desired lateral accel
	const float roll = atanf(lateral_accel / G) * RAD_TO_DEG;

	// Return clamped roll angle
	_roll_setpoint = clamp(roll, -_roll_limit, _roll_limit);
}

void L1Control::navigate_loiter()
{

}

void L1Control::navigate_heading()
{

}
