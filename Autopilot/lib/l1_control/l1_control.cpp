#include "l1_control.h"

// Add navigate_heading
// If its first waypoint, use navigate heading

void L1Control::navigate_waypoints(float x, float y, float vel_x, float vel_y, float ground_speed,
					   	   	   	   float start_x, float start_y, float end_x, float end_y)
{
	// Calculate track heading (bearing from previous to target waypoint)
	const float trk_hdg = atan2f(end_y - start_y, end_x - start_x);

	// Compute cross-track error (perpendicular distance from aircraft to path)
	const float xte = cosf(trk_hdg) * (y - end_y) - sinf(trk_hdg) * (x - end_x);

	// Calculate the L1 ratio
	float l1_ratio = 1.0f / M_PI * _l1_damping * _l1_period;

	// Calculate L1 distance and scale with speed
	const float l1_dist = fmaxf(l1_ratio * ground_speed, 1.0);

	// Calculate correction angle
	const float correction_angle = asinf(clamp(xte / l1_dist, -1, 1)); // Domain of acos is [-1, 1]

	// Apply correction angle to track heading to compute heading setpoint
	const float hdg_setpoint = trk_hdg - correction_angle;

	// Calculate plane velocity heading
	const float plane_hdg = atan2f(vel_y, vel_x);

	// Calculate plane heading error
	const float hdg_err = hdg_setpoint - plane_hdg;

	// Calculate the L1 gain
	const float k_l1 = 4.0f * _l1_damping * _l1_damping;

	// Calculate lateral acceleration using l1 guidance
	const float lateral_accel = k_l1 * ground_speed * ground_speed / l1_dist * sinf(hdg_err);

	// Calculate roll to get desired lateral accel
	const float roll = atanf(lateral_accel / G) * RAD_TO_DEG;

	// Return clamped roll angle
	_roll_setpoint = clamp(roll, -_roll_limit, _roll_limit);
}

void L1Control::navigate_loiter(float x, float y, float vel_x, float vel_y, float ground_speed,
								float target_x, float target_y, float radius, int8_t direction)
{
	// Calculate the gains for the PD loop
	float omega = (2.0f * M_PI / _l1_period);
	float K_crosstrack = omega * omega;
	float K_velocity = 2.0f * _l1_damping * omega;

	// Calculate the L1 gain
	float k_l1 = 4.0f * _l1_damping * _l1_damping;

	// Calculate the L1 ratio
	float l1_ratio = 1.0f / M_PI * _l1_damping * _l1_period;

	// Calculate the L1 length required for the desired period
	float l1_dist = l1_ratio * ground_speed;

	// Compute vector from loiter center to aircraft position
	float dx = x - target_x;
	float dy = y - target_y;
	float dist_to_center = sqrtf(dx * dx + dy * dy);

	// Normalize the radial vector to get unit vector
	float norm = fmaxf(dist_to_center, 0.001); // Prevent division by zero
	float unit_dx = dx / norm;
	float unit_dy = dy / norm;

	// Compute radial and tangential components of velocity
	float radial_vel = vel_x * unit_dx + vel_y * unit_dy;
	float tangential_vel = direction * (vel_x * unit_dy - vel_y * unit_dx); // CCW is positive

	// Cross-track error (distance from loiter radius)
	float xtrack_error = dist_to_center - radius;

	printf("loiter xte: %f\n", xtrack_error);
	printf("dist_to_center: %f\n", dist_to_center);
	printf("radial_vel: %f\n", radial_vel);

	// PD control
	float lateral_accel_pd = xtrack_error * K_crosstrack + radial_vel * K_velocity;

	// Prevent PD output from turning the wrong way when in circle mode
	if (tangential_vel < 0.0f && _circle_mode) {
	    lateral_accel_pd = fmaxf(lateral_accel_pd, 0.0f);
	}

	// Centripetal acceleration to stay on circle
	float centripetal_accel = (tangential_vel * tangential_vel) / fmaxf(radius + xtrack_error, 0.5f * radius);

	// Combine both
	float lateral_accel = direction * (lateral_accel_pd + centripetal_accel);

	 // Compute error track angle (eta) between velocity vector and radial vector
	float xtrack_vel = unit_dx * vel_y - unit_dy * vel_x;
	float ltrack_vel = -(unit_dx * vel_x + unit_dy * vel_y);
	float eta = atan2f(xtrack_vel, ltrack_vel);
	eta = fminf(fmaxf(eta, -M_PI / 2.0f), M_PI / 2.0f);

	// Decide mode: circle vs capture
	float accel_capture = k_l1 * ground_speed * ground_speed / l1_dist * sinf(eta);
	bool outside_circle = xtrack_error > 0.0f;

	if (((accel_capture < lateral_accel && direction > 0) ||
		 (accel_capture > lateral_accel && direction < 0)) &&
		 outside_circle)
	{
		lateral_accel = accel_capture;
		_circle_mode = false;
	}
	else
	{
		_circle_mode = true;
	}

	// Calculate roll to get desired lateral accel
	const float roll = atanf(lateral_accel / G) * RAD_TO_DEG;

	printf("l1 roll: %f\n", roll);

	// Return clamped roll angle
	_roll_setpoint = clamp(roll, -_roll_limit, _roll_limit);
}
