#include "modules/l1_controller/l1_controller.h"

// S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
// Proceedings of the AIAA Guidance, Navigation and Control
// Conference, Aug 2004. AIAA-2004-4900.

L1_controller::L1_controller(HAL* hal, Plane* plane)
	: Module(hal, plane)
{
}

void L1_controller::update()
{
	if (_plane->system_mode == System_mode::FLIGHT &&
		_plane->auto_mode != Auto_mode::TOUCHDOWN)
	{
		switch (_plane->flight_mode)
		{
		case Flight_mode::MANUAL:
			update_mission();
			break;
		case Flight_mode::AUTO:
			handle_auto_mode();
			break;
		}
	}
}

void L1_controller::handle_auto_mode()
{
	switch (_plane->auto_mode)
	{
	case Auto_mode::TAKEOFF:
	case Auto_mode::MISSION:
	case Auto_mode::LAND:
		update_mission();
		break;
	case Auto_mode::FLARE:
		update_flare();
		break;
	case Auto_mode::TOUCHDOWN:
		break;
	}
}

// Update roll and altitude setpoints
void L1_controller::update_mission()
{
	// Determine target and previous waypoints
	const Waypoint& prev_wp = _plane->waypoints[_plane->waypoint_index - 1];
	const Waypoint& target_wp = _plane->waypoints[_plane->waypoint_index];

	// Convert waypoints to north east coordinates
	double prev_north, prev_east, tgt_north, tgt_east;
	lat_lon_to_meters(_plane->waypoints[0].lat, _plane->waypoints[0].lon, prev_wp.lat, prev_wp.lon,
					  &prev_north, &prev_east);
	lat_lon_to_meters(_plane->waypoints[0].lat, _plane->waypoints[0].lon, target_wp.lat, target_wp.lon,
					  &tgt_north, &tgt_east);

	// Calculate track heading (bearing from previous to target waypoint)
	const float trk_hdg = atan2f(tgt_east - prev_east, tgt_north - prev_north);

	// Compute cross-track error (perpendicular distance from aircraft to path)
	const float rel_east = _plane->nav_pos_east - tgt_east;
	const float rel_north = _plane->nav_pos_north - tgt_north;
	const float xte = cosf(trk_hdg) * rel_east - sinf(trk_hdg) * rel_north;

	// Scale L1 distance with speed
	const float l1_dist = max(get_params()->l1_ctrl.period * _plane->nav_gnd_spd / M_PI, 1.0);

	// Calculate correction angle
	const float correction_angle = asinf(clamp(xte / l1_dist, -1, 1)); // Domain of acos is [-1, 1]
	const float hdg_setpoint = trk_hdg - correction_angle;

	// Calculate plane heading error
	const float hdg_err = hdg_setpoint - wrap_pi(_plane->get_ahrs_data(ahrs_handle).yaw * DEG_TO_RAD);

	// Calculate roll setpoint using l1 guidance
	const float lateral_accel = (2 * powf(_plane->nav_gnd_spd, 2) / l1_dist) * sinf(hdg_err);
	_plane->roll_setpoint = calculate_roll_setpoint(lateral_accel);
	_plane->guidance_d_setpoint = calculate_altitude_setpoint(prev_north, prev_east,
															  tgt_north, tgt_east,
															  prev_wp, target_wp);

}

// Decrease altitude setpoint at the flare sink rate and set roll to 0
void L1_controller::update_flare()
{
	const Waypoint& land_wp = _plane->waypoints[_plane->waypoint_index];
	const Waypoint& appr_wp = _plane->waypoints[_plane->waypoint_index - 1];

	// Calculate the glideslope angle based on the altitude difference and horizontal distance
	float dist_land_appr = lat_lon_to_distance(land_wp.lat, land_wp.lon, appr_wp.lat, appr_wp.lon);
	float glideslope_angle = atan2f(land_wp.alt - appr_wp.alt, dist_land_appr);

	// Linearly interpolate the sink rate based on the current altitude and flare parameters
	float final_altitude = 0;
	float final_sink_rate = get_params()->landing.flare_sink_rate;
	float initial_altitude = max(get_params()->landing.flare_alt, final_altitude);
	float initial_sink_rate = max(get_params()->tecs.aspd_land * sinf(glideslope_angle),
								  final_sink_rate);
	float sink_rate = lerp(
		initial_altitude, initial_sink_rate,
		final_altitude, final_sink_rate,
		clamp(-_plane->nav_pos_down, final_altitude, initial_altitude)
	);

	// Update the guidance setpoint with the calculated sink rate
	_plane->guidance_d_setpoint += sink_rate * _plane->dt_s;
	_plane->roll_setpoint = 0;
}

float L1_controller::calculate_altitude_setpoint(float prev_north, float prev_east,
		  	  	  	  	  	  	  	  	  	  	 float tgt_north, float tgt_east,
												 Waypoint prev_wp, Waypoint target_wp)
{
	// Takeoff
	if (_plane->waypoint_index == 1)
	{
		return target_wp.alt;
	}

	float dist_prev_tgt = distance(prev_north, prev_east, tgt_north, tgt_east);

	float along_track_dist = compute_along_track_distance(
		prev_north, prev_east, tgt_north, tgt_east,
		_plane->nav_pos_north, _plane->nav_pos_east
	);

	float initial_dist = get_params()->navigator.min_dist_wp;
	float final_dist;
	if (_plane->waypoint_index == _plane->num_waypoints - 1)
	{
		// During landing, go directly to landing point
		final_dist =  dist_prev_tgt;
	}
	else
	{
		// Reach altitude when within acceptance radius of next waypoint
		final_dist = dist_prev_tgt - get_params()->navigator.min_dist_wp;
	}

	// Altitude first order hold
	return lerp(
		initial_dist, prev_wp.alt,
		final_dist, target_wp.alt,
		clamp(along_track_dist, initial_dist, final_dist)
	);
}

float L1_controller::calculate_roll_setpoint(float lateral_accel) const
{
	float roll = atanf(lateral_accel / G) * RAD_TO_DEG;

	if (_plane->auto_mode == Auto_mode::TAKEOFF)
	{
		roll = clamp(roll, -get_params()->takeoff.roll_lim, get_params()->takeoff.roll_lim);
	}
	else
	{
		roll = clamp(roll, -get_params()->l1_ctrl.roll_lim, get_params()->l1_ctrl.roll_lim);
	}

	return roll;
}

// Helper function to compute along-track distance (projected aircraft position onto path)
float L1_controller::compute_along_track_distance(float start_n, float start_e,
												  float end_n, float end_e,
											 	  float pos_n, float pos_e)
{
	float vec_north = end_n - start_n;
	float vec_east = end_e - start_e;
	float vec_norm = sqrtf(vec_north * vec_north + vec_east * vec_east);

	if (vec_norm == 0) return 0.0f;

	float proj_factor = ((pos_n - start_n) * vec_north + (pos_e - start_e) * vec_east) /
						(vec_norm * vec_norm);
	return proj_factor * vec_norm;
}

// Helper function to compute Euclidean distance
float L1_controller::distance(float n1, float e1, float n2, float e2)
{
	float dn = n2 - n1;
	float de = e2 - e1;
	return sqrtf(dn * dn + de * de);
}
