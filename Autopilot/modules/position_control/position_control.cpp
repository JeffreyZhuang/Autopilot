#include <modules/position_control/position_control.h>

// S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
// Proceedings of the AIAA Guidance, Navigation and Control
// Conference, Aug 2004. AIAA-2004-4900.

PositionControl::PositionControl(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _ahrs_sub(data_bus->ahrs_node),
	  _pos_est_sub(data_bus->pos_est_node),
	  _modes_sub(data_bus->modes_node),
	  _telem_sub(data_bus->telem_node),
	  _navigator_sub(data_bus->navigator_node),
	  _rc_sub(data_bus->rc_node),
	  _time_sub(data_bus->time_node),
	  _position_control_pub(data_bus->position_control_node)
{
}

void PositionControl::update()
{
	_ahrs_data = _ahrs_sub.get();
	_pos_est_data = _pos_est_sub.get();
	_modes_data = _modes_sub.get();
	_telem_data = _telem_sub.get();
	_navigator_data = _navigator_sub.get();
	_rc_data = _rc_sub.get();

	if (_modes_data.system_mode == System_mode::FLIGHT)
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

	_position_control_pub.publish(_position_control);
}

void PositionControl::handle_manual_mode()
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

void PositionControl::update_stabilized()
{
	_position_control.roll_setpoint = _rc_data.ail_norm * param_get_float(L1_ROLL_LIM);
}

void PositionControl::handle_auto_mode()
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
	}
}

// Update roll and altitude setpoints
void PositionControl::update_mission()
{
	// Determine target and previous waypoints
	const Waypoint& prev_wp = _telem_data.waypoints[_navigator_data.waypoint_index - 1];
	const Waypoint& target_wp = _telem_data.waypoints[_navigator_data.waypoint_index];

	// Convert waypoints to north east coordinates
	double prev_north, prev_east, tgt_north, tgt_east;
	lat_lon_to_meters(_telem_data.waypoints[0].lat, _telem_data.waypoints[0].lon,
					  prev_wp.lat, prev_wp.lon, &prev_north, &prev_east);
	lat_lon_to_meters(_telem_data.waypoints[0].lat, _telem_data.waypoints[0].lon,
					  target_wp.lat, target_wp.lon, &tgt_north, &tgt_east);

	// Calculate track heading (bearing from previous to target waypoint)
	const float trk_hdg = atan2f(tgt_east - prev_east, tgt_north - prev_north);

	// Compute cross-track error (perpendicular distance from aircraft to path)
	const float rel_east = _pos_est_data.pos_e - tgt_east;
	const float rel_north = _pos_est_data.pos_n - tgt_north;
	const float xte = cosf(trk_hdg) * rel_east - sinf(trk_hdg) * rel_north;

	// Calculate L1 distance and scale with speed
	const float l1_dist = fmaxf(param_get_float(L1_PERIOD) * _pos_est_data.gnd_spd / M_PI, 1.0);

	// Calculate correction angle
	const float correction_angle = asinf(clamp(xte / l1_dist, -1, 1)); // Domain of acos is [-1, 1]
	const float hdg_setpoint = trk_hdg - correction_angle;

	// Calculate plane heading error
	const float hdg_err = hdg_setpoint - _ahrs_data.yaw * DEG_TO_RAD;

	// Calculate lateral acceleration using l1 guidance
	const float lateral_accel = 2 * powf(_pos_est_data.gnd_spd, 2) / l1_dist * sinf(hdg_err);

	// Update roll and altitude setpoints
	_position_control.roll_setpoint = calculate_roll_setpoint(lateral_accel);
	_position_control.d_setpoint = calculate_altitude_setpoint(prev_north, prev_east,
													  tgt_north, tgt_east,
													  prev_wp, target_wp);
}

// Decrease altitude setpoint at the flare sink rate and set roll to 0
void PositionControl::update_flare()
{
	const Waypoint& land_wp = _telem_data.waypoints[-1];
	const Waypoint& appr_wp = _telem_data.waypoints[-2];

	// Calculate the glideslope angle based on the altitude difference and horizontal distance
	const float dist_land_appr = lat_lon_to_distance(land_wp.lat, land_wp.lon,
													 appr_wp.lat, appr_wp.lon);
	const float glideslope_angle = atan2f(land_wp.alt - appr_wp.alt,
										  dist_land_appr - param_get_float(NAV_ACC_RAD));

	// Linearly interpolate the sink rate based on the current altitude and flare parameters
	const float final_altitude = 0;
	const float final_sink_rate = param_get_float(LND_FL_SINK);
	const float initial_altitude = fmaxf(param_get_float(LND_FL_ALT), final_altitude);
	const float initial_sink_rate = fmaxf(param_get_float(LND_SPD)* sinf(glideslope_angle),
								  	  	  final_sink_rate);
	const float sink_rate = lerp(
		initial_altitude, initial_sink_rate,
		final_altitude, final_sink_rate,
		clamp(-_pos_est_data.pos_d, final_altitude, initial_altitude)
	);

	// Update altitude setpoint with the calculated sink rate
	_position_control.d_setpoint += sink_rate * _time_data.dt_s;
	_position_control.roll_setpoint = 0;
}

float PositionControl::calculate_altitude_setpoint(const float prev_north, const float prev_east, const float prev_down,
		  	  	  	  	  	  	  	  	  	  	   const float tgt_north, const float tgt_east, const float tgt_down)
{
	if (_navigator_data.waypoint_index == 1)
	{
		// Takeoff
		return tgt_down;
	}

	const float dist_prev_tgt = distance(prev_north, prev_east, tgt_north, tgt_east);
	const float along_track_dist = compute_along_track_distance(
		prev_north, prev_east, tgt_north, tgt_east,
		_pos_est_data.pos_n, _pos_est_data.pos_e
	);

	const float initial_dist = param_get_float(NAV_ACC_RAD);
	float final_dist;

	if (_navigator_data.waypoint_index == _telem_data.num_waypoints - 1)
	{
		// During landing, go directly to landing point
		final_dist = dist_prev_tgt;
	}
	else
	{
		// Reach altitude when within acceptance radius of next waypoint
		final_dist = dist_prev_tgt - param_get_float(NAV_ACC_RAD);
	}

	// Altitude first order hold
	return lerp(initial_dist, prev_down,
		final_dist, tgt_down,
		clamp(along_track_dist, initial_dist, final_dist)
	);
}

float PositionControl::calculate_roll_setpoint(float lateral_accel) const
{
	const float roll = atanf(lateral_accel / G) * RAD_TO_DEG;

	if (_modes_data.auto_mode == Auto_mode::TAKEOFF)
	{
		return 0; // Keep wings level during takeoff
	}
	else
	{
		return clamp(roll, -param_get_float(L1_ROLL_LIM), param_get_float(L1_ROLL_LIM));
	}
}

// Helper function to compute along-track distance (projected aircraft position onto path)
float PositionControl::compute_along_track_distance(float start_n, float start_e,
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
float PositionControl::distance(float n1, float e1, float n2, float e2)
{
	const float dn = n2 - n1;
	const float de = e2 - e1;
	return sqrtf(dn * dn + de * de);
}

// wb: weight balance
// wb = 0: only spd
// wb = 1: balanced
// wb = 2: only alt
void PositionControl::tecs_calculate_energies(float target_vel_mps, float target_alt_m, float wb)
{
	// Calculate specific energy
	// SPe = gh
	// SKe = 1/2 v^2
	// Ignore mass since its the energy ratio that matters
	float energy_pot = G * (-_pos_est_data.pos_d);
	float energy_kin = 0.5 * powf(_pos_est_data.gnd_spd, 2);
	float energy_total = energy_pot + energy_kin;

	// Calculate target energy using same equations
	float target_pot = G * (-target_alt_m);
	float target_kin = 0.5 * powf(target_vel_mps, 2);
	float target_total = target_pot + target_kin;

	// Clamp total energy setpoint within allowed airspeed range
	// Prevent stall/overspeed
	float min_kin = 0.5 * powf(param_get_float(MIN_SPD), 2);
	float max_kin = 0.5 * powf(param_get_float(MAX_SPD), 2);
	target_total = clamp(target_total, energy_pot + min_kin, energy_pot + max_kin);

	// Compute energy difference setpoint and measurement
	float energy_diff_setpoint = wb * target_pot - (2.0 - wb) * target_kin;
	float energy_diff = wb * energy_pot - (2.0 - wb) * energy_kin;

	// Clamp energy balance within allowed range
	float min_diff = wb * target_pot - (2.0f - wb) * max_kin;
	float max_diff = wb * target_pot - (2.0f - wb) * min_kin;
	energy_diff_setpoint = clamp(energy_diff_setpoint, min_diff, max_diff);

	_total_energy_setpoint = target_total;
	_total_energy = energy_total;
	_energy_balance_setpoint = energy_diff_setpoint;
	_energy_balance = energy_diff;
}

float PositionControl::tecs_control_energy_balance()
{
	return energy_balance_controller.get_output(
		_energy_balance,
		_energy_balance_setpoint,
		param_get_float(TECS_PTCH_KP),
		param_get_float(TECS_PTCH_KI),
		param_get_float(TECS_PTCH_LIM),
		-param_get_float(TECS_PTCH_LIM),
		param_get_float(TECS_PTCH_LIM),
		0,
		_time_data.dt_s
	);
}

float PositionControl::tecs_control_total_energy()
{
	return total_energy_controller.get_output(
		_total_energy,
		_total_energy_setpoint,
		param_get_float(TECS_THR_KP),
		param_get_float(TECS_THR_KI),
		1,
		0,
		1,
		param_get_float(MIS_THR),
		_time_data.dt_s
	);
}
