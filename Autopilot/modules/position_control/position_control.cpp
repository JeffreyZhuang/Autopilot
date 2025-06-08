#include "position_control.h"

// S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
// Proceedings of the AIAA Guidance, Navigation and Control
// Conference, Aug 2004. AIAA-2004-4900.

PositionControl::PositionControl(HAL* hal, DataBus* data_bus)
	: Module(hal, data_bus),
	  _ahrs_sub(data_bus->ahrs_node),
	  _local_pos_sub(data_bus->local_position_node),
	  _modes_sub(data_bus->modes_node),
	  _waypoint_sub(data_bus->waypoint_node),
	  _rc_sub(data_bus->rc_node),
	  _position_control_pub(data_bus->position_control_node)
{
}

void PositionControl::update()
{
	const uint64_t time = _hal->get_time_us();
	_dt = clamp((time - _last_time) * US_TO_S, DT_MIN, DT_MAX);
	_last_time = time;

	_ahrs_data = _ahrs_sub.get();
	_local_pos = _local_pos_sub.get();
	_modes_data = _modes_sub.get();
	_waypoint = _waypoint_sub.get();
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
		update_direct();
		break;
	case Manual_mode::STABILIZED:
		update_stabilized();
		break;
	}
}

void PositionControl::update_direct()
{
	_position_control.roll_setpoint = 0;
	_position_control.pitch_setpoint = 0;
	_position_control.throttle_setpoint = _rc_data.thr_norm;
}

void PositionControl::update_stabilized()
{
	float roll_lim, ptch_lim;

	param_get(L1_ROLL_LIM, &roll_lim);
	param_get(TECS_PTCH_LIM, &ptch_lim);

	_position_control.roll_setpoint = _rc_data.ail_norm * roll_lim;
	_position_control.pitch_setpoint = _rc_data.ele_norm * ptch_lim;
	_position_control.throttle_setpoint = _rc_data.thr_norm;
}

void PositionControl::handle_auto_mode()
{
	switch (_modes_data.auto_mode)
	{
	case Auto_mode::TAKEOFF:
		update_takeoff();
		break;
	case Auto_mode::MISSION:
		update_mission();
	case Auto_mode::LAND:
		update_land();
		break;
	case Auto_mode::FLARE:
		update_flare();
		break;
	}
}

void PositionControl::update_takeoff()
{
	float takeoff_pitch;

	param_get(TKO_PTCH, &takeoff_pitch);

	_position_control.roll_setpoint = 0;
	_position_control.pitch_setpoint = takeoff_pitch;
	_position_control.throttle_setpoint = _rc_data.thr_norm;
}

// Update roll and altitude setpoints
void PositionControl::update_mission()
{
	switch (mission_get().mission_type)
	{
	case MISSION_LOITER:
		break;
	case MISSION_WAYPOINT:
		break;
	case MISSION_LAND:
		break;
	}

	float cruise_speed;

	param_get(MIS_SPD, &cruise_speed);

	// Update roll setpoint
	_position_control.roll_setpoint = l1_calculate_roll();

	// Calculate altitude setpoint
	_d_setpoint = mission_get_altitude();

	// Update TECS
	tecs_calculate_energies(cruise_speed, _d_setpoint, 1);
	_position_control.pitch_setpoint = tecs_control_energy_balance();
	_position_control.throttle_setpoint = tecs_control_total_energy();
}

void PositionControl::update_land()
{
	float landing_speed;

	param_get(LND_SPD, &landing_speed);

	// Update roll setpoint
	_position_control.roll_setpoint = l1_calculate_roll();

	// Follow glideslope
	const float along_track_dist = compute_along_track_distance(
		_waypoint.previous_north, _waypoint.previous_east,
		_waypoint.current_north, _waypoint.current_east,
		_local_pos.x, _local_pos.y
	);

	_d_setpoint = along_track_dist * tanf(mission_get().glideslope_angle * DEG_TO_RAD);

	// Update TECS
	tecs_calculate_energies(landing_speed, _d_setpoint, 1);
	_position_control.pitch_setpoint = tecs_control_energy_balance();
	_position_control.throttle_setpoint = tecs_control_total_energy();
}

// Decrease altitude setpoint at the flare sink rate and set roll to 0
void PositionControl::update_flare()
{
	float acceptance_radius, flare_sink_rate, flare_alt,
		  landing_speed;

	param_get(NAV_ACC_RAD, &acceptance_radius);
	param_get(LND_FL_SINK, &flare_sink_rate);
	param_get(LND_FL_ALT, &flare_alt);
	param_get(LND_SPD, &landing_speed);

	// Calculate the glideslope angle based on the altitude difference and horizontal distance
	const float dist_land_appr = distance(_waypoint.previous_north, _waypoint.previous_east,
										  _waypoint.current_north, _waypoint.current_east);
	const float glideslope_angle = atan2f(mission_get_altitude(), dist_land_appr - acceptance_radius);

	// Linearly interpolate the sink rate based on the current altitude and flare parameters
	const float final_altitude = 0;
	const float final_sink_rate = flare_sink_rate;
	const float initial_altitude = fmaxf(flare_alt, final_altitude);
	const float initial_sink_rate = fmaxf(landing_speed * sinf(glideslope_angle),
								  	  	  final_sink_rate);
	const float sink_rate = lerp(initial_altitude, initial_sink_rate,
								 final_altitude, final_sink_rate,
								 clamp(-_local_pos.z, final_altitude, initial_altitude));

	// Update altitude setpoint with the calculated sink rate
	_d_setpoint += sink_rate * _dt;
	_position_control.roll_setpoint = 0;

	tecs_calculate_energies(0, _d_setpoint, 2);
	_position_control.pitch_setpoint = tecs_control_energy_balance();
	_position_control.throttle_setpoint = 0;
}

float PositionControl::l1_calculate_roll() const
{
	float l1_period, roll_lim;

	param_get(L1_PERIOD, &l1_period);
	param_get(L1_ROLL_LIM, &roll_lim);

	// Calculate track heading (bearing from previous to target waypoint)
	const float trk_hdg = atan2f(_waypoint.current_east - _waypoint.previous_east,
								 _waypoint.current_north - _waypoint.previous_north);

	// Compute cross-track error (perpendicular distance from aircraft to path)
	const float rel_east = _local_pos.y - _waypoint.current_east;
	const float rel_north = _local_pos.x - _waypoint.current_north;
	const float xte = cosf(trk_hdg) * rel_east - sinf(trk_hdg) * rel_north;

	// Calculate L1 distance and scale with speed
	const float l1_dist = fmaxf(l1_period * _local_pos.gnd_spd / M_PI, 1.0);

	// Calculate correction angle
	const float correction_angle = asinf(clamp(xte / l1_dist, -1, 1)); // Domain of acos is [-1, 1]

	// Apply correction angle to track heading to compute heading setpoint
	const float hdg_setpoint = trk_hdg - correction_angle;

	// Calculate plane velocity heading
	const float plane_hdg = atan2f(_local_pos.vy, _local_pos.vx);

	// Calculate plane heading error
	const float hdg_err = hdg_setpoint - plane_hdg;

	// Calculate lateral acceleration using l1 guidance
	const float lateral_accel = 2 * powf(_local_pos.gnd_spd, 2) / l1_dist * sinf(hdg_err);

	// Calculate roll to get desired lateral accel
	const float roll = atanf(lateral_accel / G) * RAD_TO_DEG;

	// Return clamped roll angle
	return clamp(roll, -roll_lim, roll_lim);
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

// wb: weight balance
// wb = 0: only spd
// wb = 1: balanced
// wb = 2: only alt
void PositionControl::tecs_calculate_energies(float target_vel_mps, float target_alt_m, float wb)
{
	float min_spd, max_spd;

	param_get(MIN_SPD, &min_spd);
	param_get(MAX_SPD, &max_spd);

	// Calculate specific energy
	// SPe = gh
	// SKe = 1/2 v^2
	// Ignore mass since its the energy ratio that matters
	float energy_pot = G * (-_local_pos.z);
	float energy_kin = 0.5 * powf(_local_pos.gnd_spd, 2);
	float energy_total = energy_pot + energy_kin;

	// Calculate target energy using same equations
	float target_pot = G * (-target_alt_m);
	float target_kin = 0.5 * powf(target_vel_mps, 2);
	float target_total = target_pot + target_kin;

	// Clamp total energy setpoint within allowed airspeed range
	// Prevent stall/overspeed
	float min_kin = 0.5 * powf(min_spd, 2);
	float max_kin = 0.5 * powf(max_spd, 2);
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
	float kP, kI, ptch_lim;

	param_get(TECS_PTCH_KP, &kP);
	param_get(TECS_PTCH_KI, &kI);
	param_get(TECS_PTCH_LIM, &ptch_lim);

	return energy_balance_controller.get_output(
		_energy_balance, _energy_balance_setpoint,
		kP, kI, ptch_lim, -ptch_lim, ptch_lim, 0, _dt
	);
}

float PositionControl::tecs_control_total_energy()
{
	float kP, kI, cruise_thr_trim;

	param_get(TECS_THR_KP, &kP);
	param_get(TECS_THR_KI, &kI);
	param_get(MIS_THR, &cruise_thr_trim);

	return total_energy_controller.get_output(
		_total_energy, _total_energy_setpoint,
		kP, kI, 1, 0, 1, cruise_thr_trim, _dt
	);
}
