#include "modules/l1_controller/l1_controller.h"

// S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
// Proceedings of the AIAA Guidance, Navigation and Control
// Conference, Aug 2004. AIAA-2004-4900.

L1_controller::L1_controller(HAL* hal)
	: Module(hal),
	  _ahrs_sub(Data_bus::get_instance().ahrs_data),
	  _pos_est_sub(Data_bus::get_instance().pos_est_data),
	  _modes_sub(Data_bus::get_instance().modes_data),
	  _telem_sub(Data_bus::get_instance().telem_data),
	  _navigator_sub(Data_bus::get_instance().navigator_data),
	  _rc_sub(Data_bus::get_instance().rc_data),
	  _time_sub(Data_bus::get_instance().time_data),
	  _l1_pub(Data_bus::get_instance().l1_data)
{
}

void L1_controller::update()
{
	_ahrs_data = _ahrs_sub.get();
	_pos_est_data = _pos_est_sub.get();
	_modes_data = _modes_sub.get();
	_telem_data = _telem_sub.get();
	_navigator_data = _navigator_sub.get();
	_rc_data = _rc_sub.get();

	if (_modes_data.system_mode == System_mode::FLIGHT &&
		_modes_data.auto_mode != Auto_mode::TOUCHDOWN)
	{
		switch (_modes_data.flight_mode)
		{
		case Flight_mode::MANUAL:
			handle_manual_mode();
			break;
		case Flight_mode::AUTO:
			handle_auto_mode();
			break;
		}
	}

	_l1_pub.publish(_l1_data);
}

void L1_controller::handle_manual_mode()
{
	switch (_modes_data.manual_mode)
	{
	case Manual_mode::DIRECT:
		break;
	case Manual_mode::STABILIZED:
		update_stabilized();
		break;
	}
}

void L1_controller::update_stabilized()
{
	_l1_data.roll_setpoint = _rc_data.ail_norm * get_params()->att_ctrl.fbw_roll_lim;
}

void L1_controller::handle_auto_mode()
{
	switch (_modes_data.auto_mode)
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
	const Waypoint& prev_wp = _telem_data.waypoints[_navigator_data.waypoint_index - 1];
	const Waypoint& target_wp = _telem_data.waypoints[_navigator_data.waypoint_index];

	// Convert waypoints to north east coordinates
	double prev_north, prev_east, tgt_north, tgt_east;
	lat_lon_to_meters(Data_bus::get_instance().get_home().lat, Data_bus::get_instance().get_home().lon,
					  prev_wp.lat, prev_wp.lon, &prev_north, &prev_east);
	lat_lon_to_meters(Data_bus::get_instance().get_home().lat, Data_bus::get_instance().get_home().lon,
					  target_wp.lat, target_wp.lon, &tgt_north, &tgt_east);

	// Calculate track heading (bearing from previous to target waypoint)
	const float trk_hdg = atan2f(tgt_east - prev_east, tgt_north - prev_north);

	// Compute cross-track error (perpendicular distance from aircraft to path)
	const float rel_east = _pos_est_data.pos_e - tgt_east;
	const float rel_north = _pos_est_data.pos_n - tgt_north;
	const float xte = cosf(trk_hdg) * rel_east - sinf(trk_hdg) * rel_north;

	// Calculate L1 distance and scale with speed
	const float l1_dist = fmaxf(get_params()->l1_ctrl.period * _pos_est_data.gnd_spd / M_PI, 1.0);

	// Calculate correction angle
	const float correction_angle = asinf(clamp(xte / l1_dist, -1, 1)); // Domain of acos is [-1, 1]
	const float hdg_setpoint = trk_hdg - correction_angle;

	// Calculate plane heading error
	const float hdg_err = hdg_setpoint - wrap_pi(_ahrs_data.yaw * DEG_TO_RAD);

	// Calculate lateral acceleration using l1 guidance
	const float lateral_accel = 2 * powf(_pos_est_data.gnd_spd, 2) / l1_dist * sinf(hdg_err);

	// Update roll and altitude setpoints
	_l1_data.roll_setpoint = calculate_roll_setpoint(lateral_accel);
	_l1_data.d_setpoint = calculate_altitude_setpoint(prev_north, prev_east,
															  tgt_north, tgt_east,
															  prev_wp, target_wp);
}

// Decrease altitude setpoint at the flare sink rate and set roll to 0
void L1_controller::update_flare()
{
	const Waypoint& land_wp = _telem_data.waypoints[-1];
	const Waypoint& appr_wp = _telem_data.waypoints[-2];

	// Calculate the glideslope angle based on the altitude difference and horizontal distance
	const float dist_land_appr = lat_lon_to_distance(land_wp.lat, land_wp.lon,
													 appr_wp.lat, appr_wp.lon);
	const float glideslope_angle = atan2f(land_wp.alt - appr_wp.alt,
										  dist_land_appr - get_params()->navigator.min_dist_wp);

	// Linearly interpolate the sink rate based on the current altitude and flare parameters
	const float final_altitude = 0;
	const float final_sink_rate = get_params()->landing.flare_sink_rate;
	const float initial_altitude = fmaxf(get_params()->landing.flare_alt, final_altitude);
	const float initial_sink_rate = fmaxf(get_params()->tecs.aspd_land * sinf(glideslope_angle),
								  final_sink_rate);
	const float sink_rate = lerp(
		initial_altitude, initial_sink_rate,
		final_altitude, final_sink_rate,
		clamp(-_pos_est_data.pos_d, final_altitude, initial_altitude)
	);

	// Update altitude setpoint with the calculated sink rate
	_l1_data.d_setpoint += sink_rate * _time_data.dt_s;
	_l1_data.roll_setpoint = 0;
}

float L1_controller::calculate_altitude_setpoint(const float prev_north, const float prev_east,
		  	  	  	  	  	  	  	  	  	  	 const float tgt_north, const float tgt_east,
												 const Waypoint& prev_wp, const Waypoint& target_wp)
{
	if (_navigator_data.waypoint_index == 1)
	{
		// Takeoff
		return target_wp.alt;
	}

	const float dist_prev_tgt = distance(prev_north, prev_east, tgt_north, tgt_east);
	const float along_track_dist = compute_along_track_distance(
		prev_north, prev_east, tgt_north, tgt_east,
		_pos_est_data.pos_n, _pos_est_data.pos_e
	);

	const float initial_dist = get_params()->navigator.min_dist_wp;
	float final_dist;

	if (_navigator_data.waypoint_index == _telem_data.num_waypoints - 1)
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
	const float roll = atanf(lateral_accel / G) * RAD_TO_DEG;

	if (_modes_data.auto_mode == Auto_mode::TAKEOFF)
	{
		return clamp(roll, -get_params()->takeoff.roll_lim, get_params()->takeoff.roll_lim);
	}
	else
	{
		return clamp(roll, -get_params()->l1_ctrl.roll_lim, get_params()->l1_ctrl.roll_lim);
	}
}

// Helper function to compute along-track distance (projected aircraft position onto path)
float L1_controller::compute_along_track_distance(float start_n, float start_e,
												  float end_n, float end_e,
											 	  float pos_n, float pos_e)
{
	const float vec_north = end_n - start_n;
	const float vec_east = end_e - start_e;
	const float vec_norm = sqrtf(vec_north * vec_north + vec_east * vec_east);

	if (vec_norm == 0) return 0.0f;

	const float proj_factor = ((pos_n - start_n) * vec_north + (pos_e - start_e) * vec_east) /
							  (vec_norm * vec_norm);
	return proj_factor * vec_norm;
}

// Helper function to compute Euclidean distance
float L1_controller::distance(float n1, float e1, float n2, float e2)
{
	const float dn = n2 - n1;
	const float de = e2 - e1;
	return sqrtf(dn * dn + de * de);
}
