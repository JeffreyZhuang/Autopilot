#include "position_control.h"

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

void PositionControl::update_time()
{
	const uint64_t time = _hal->get_time_us();
	_dt = clamp((time - _last_time) * US_TO_S, DT_MIN, DT_MAX);
	_last_time = time;
}

void PositionControl::poll_vehicle_data()
{
	_ahrs_data = _ahrs_sub.get();
	_local_pos = _local_pos_sub.get();
	_modes_data = _modes_sub.get();
	_waypoint = _waypoint_sub.get();
	_rc_data = _rc_sub.get();
}

void PositionControl::update_parameters()
{
	param_get(L1_ROLL_LIM, &_roll_limit);
	param_get(TECS_PTCH_LIM, &_pitch_limit);
	param_get(TKO_PTCH, &_takeoff_pitch);
	param_get(MIS_SPD, &_cruise_speed);
	param_get(LND_SPD, &_landing_speed);
	param_get(NAV_ACC_RAD, &_acceptance_radius);
	param_get(LND_FL_SINK, &_flare_sink_rate);
	param_get(LND_FL_ALT, &_flare_alt);

	TECS::Param tecs_param = {0};
	param_get(MIN_SPD, &tecs_param.min_spd);
	param_get(MAX_SPD, &tecs_param.max_spd);
	param_get(TECS_PTCH_KP, &tecs_param.pitch_gain);
	param_get(TECS_PTCH_KI, &tecs_param.pitch_integral_gain);
	param_get(TECS_PTCH_LIM, &tecs_param.max_pitch);
	param_get(TECS_THR_KP, &tecs_param.throttle_gain);
	param_get(TECS_THR_KI, &tecs_param.throttle_integral_gain);
	param_get(MIS_THR, &tecs_param.throttle_trim);
	tecs_param.min_pitch = -tecs_param.max_pitch;
	tecs_param.alt_weight = 1;
	tecs_param.min_throttle = 0;
	tecs_param.max_throttle = 1;
	_tecs.set_param(tecs_param);

	float l1_period, l1_damping, roll_limit;
	param_get(L1_PERIOD, &l1_period);
	param_get(L1_DAMPING, &l1_damping);
	param_get(L1_ROLL_LIM, &roll_limit);
	_l1_control.set_l1_period(l1_period);
	_l1_control.set_l1_damping(l1_damping);
	_l1_control.set_roll_limit(roll_limit);
}

void PositionControl::update()
{
	update_time();
	poll_vehicle_data();
	update_parameters();

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

	publish_status();
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
	_position_control.throttle_setpoint = _rc_data.thr_norm;
}

void PositionControl::update_stabilized()
{
	_position_control.roll_setpoint = _rc_data.ail_norm * _roll_limit;
	_position_control.pitch_setpoint = _rc_data.ele_norm * _pitch_limit;
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
		break;
	}
}

// TODO: I should probably add pitch to throttle feedforward...
void PositionControl::update_takeoff()
{
	_tecs.set_alt_weight(0); // Speed only
	_tecs.update(0, _local_pos.gnd_spd, 0, _cruise_speed, _dt);

	_position_control.roll_setpoint = 0;
	_position_control.pitch_setpoint = _takeoff_pitch;
	_position_control.throttle_setpoint = _tecs.get_throttle_setpoint();
}

void PositionControl::update_mission()
{
	if (mission_get().mission_type != MISSION_LAND)
	{
		reset_landing_state();
	}

	switch (mission_get().mission_type)
	{
	case MISSION_EMPTY:
		break;
	case MISSION_LOITER:
		update_mission_loiter();
		break;
	case MISSION_WAYPOINT:
		update_mission_waypoint();
		break;
	case MISSION_LAND:
		update_land();
		break;
	}
}

void PositionControl::update_mission_waypoint()
{
	if (_waypoint.current_index == mission_get().num_items - 1)
	{
		int8_t direction;

		if (mission_get().loiter_direction == LOITER_LEFT)
		{
			direction = 1;
		}
		else
		{
			direction = -1;
		}

		_l1_control.navigate_loiter(_local_pos.x, _local_pos.y, _local_pos.vx, _local_pos.vy,
									_local_pos.gnd_spd, _waypoint.current_north, _waypoint.current_east,
									mission_get().loiter_radius, direction);
	}
	else
	{
		_l1_control.navigate_waypoints(_local_pos.x, _local_pos.y, _local_pos.vx, _local_pos.vy, _local_pos.gnd_spd,
									   _waypoint.previous_north, _waypoint.previous_east,
									   _waypoint.current_north, _waypoint.current_east);
	}

	_position_control.roll_setpoint = _l1_control.get_roll_setpoint();

	_tecs_setpoint.alt = mission_get_altitude();
	_tecs_setpoint.spd = _cruise_speed;
	_tecs.set_alt_weight(1);

	tecs_update_pitch_throttle();
}

void PositionControl::update_mission_loiter()
{
	int8_t direction;

	if (mission_get().loiter_direction == LOITER_LEFT)
	{
		direction = 1;
	}
	else
	{
		direction = -1;
	}

	_l1_control.navigate_loiter(_local_pos.x, _local_pos.y, _local_pos.vx, _local_pos.vy,
								_local_pos.gnd_spd, _waypoint.current_north, _waypoint.current_east,
								mission_get().loiter_radius, direction);

	_position_control.roll_setpoint = _l1_control.get_roll_setpoint();

	_tecs_setpoint.alt = mission_get_altitude();
	_tecs_setpoint.spd = _cruise_speed;
	_tecs.set_alt_weight(1);

	tecs_update_pitch_throttle();
}

void PositionControl::update_land()
{
	switch (_landing_state)
	{
	case LandingState::LOITER:
		update_land_loiter();
		break;
	case LandingState::GLIDESLOPE:
		update_land_glideslope();
		break;
	case LandingState::FLARE:
		update_land_flare();
		break;
	}
}

void PositionControl::update_land_loiter()
{
	int8_t direction;

	if (mission_get().loiter_direction == LOITER_RIGHT)
	{
		direction = 1;
	}
	else
	{
		direction = -1;
	}

	// Get position of the start of the glideslope
	float glideslope_start_north, glideslope_start_east;

	move_point(_waypoint.current_north, _waypoint.current_east, mission_get().final_leg_dist, mission_get().runway_heading,
			   &glideslope_start_north, &glideslope_start_east);

	// Calculate heading perpendicular to the runway in the direction of the loiter point
	float perpendicular_heading = mission_get().runway_heading - 90 * direction;

	// Calculate the position of the loiter point
	float loiter_north, loiter_east;

	move_point(glideslope_start_north, glideslope_start_east, mission_get().loiter_radius, perpendicular_heading,
			   &loiter_north, &loiter_east);

	_l1_control.navigate_loiter(_local_pos.x, _local_pos.y, _local_pos.vx, _local_pos.vy,
								_local_pos.gnd_spd, loiter_north, loiter_east,
								mission_get().loiter_radius, direction);

	_position_control.roll_setpoint = _l1_control.get_roll_setpoint();

	if (_l1_control.get_circle_mode())
	{
		// Calculate loiter altitude
		float altitude_setpoint = mission_get().final_leg_dist * tanf(mission_get().glideslope_angle * DEG_TO_RAD);

		_tecs_setpoint.alt = altitude_setpoint;
		_tecs_setpoint.spd = _landing_speed;

		float heading_error = fabs(_ahrs_data.yaw - mission_get().runway_heading); // TODO: Use position or something, not yaw
		float altitude_error = fabs(-_local_pos.z - altitude_setpoint);
		float speed_error = fabs(_local_pos.gnd_spd - _landing_speed);

		if ((heading_error < 10) && (altitude_error < 1) && (speed_error < 2))
		{
			// Switch to next state
			_landing_state = LandingState::GLIDESLOPE;
		}
	}
	else
	{
		_tecs_setpoint.alt = mission_get_altitude();
		_tecs_setpoint.spd = _cruise_speed;
	}

	_tecs.set_alt_weight(1);
	tecs_update_pitch_throttle();
}

void PositionControl::update_land_glideslope()
{
	// Get position of the start of the glideslope
	float glideslope_start_north, glideslope_start_east;

	move_point(_waypoint.current_north, _waypoint.current_east, mission_get().final_leg_dist, mission_get().runway_heading,
			   &glideslope_start_north, &glideslope_start_east);

	_l1_control.navigate_waypoints(_local_pos.x, _local_pos.y, _local_pos.vx, _local_pos.vy, _local_pos.gnd_spd,
								   glideslope_start_north, glideslope_start_east,
								   _waypoint.current_north, _waypoint.current_east);

	_position_control.roll_setpoint = _l1_control.get_roll_setpoint();

	// Follow glideslope
	const float along_track_dist = compute_along_track_distance(
		_waypoint.previous_north, _waypoint.previous_east,
		_waypoint.current_north, _waypoint.current_east,
		_local_pos.x, _local_pos.y
	);

	float z_setpoint = along_track_dist * tanf(mission_get().glideslope_angle * DEG_TO_RAD);

	_tecs_setpoint.alt = -z_setpoint;
	_tecs_setpoint.spd = _landing_speed;
	_tecs.set_alt_weight(1);
	tecs_update_pitch_throttle();

	// Detect flare
	if (-_local_pos.z < _flare_alt)
	{
		_flare_alt_setpoint = _flare_alt;
		_landing_state = LandingState::FLARE;
	}
}

void PositionControl::update_land_flare()
{
	const float glideslope_sink_rate = _landing_speed * sinf(mission_get().glideslope_angle * DEG_TO_RAD);
	const float initial_altitude = fmaxf(_flare_alt, 0);
	const float initial_sink_rate = fmaxf(glideslope_sink_rate, _flare_sink_rate);
	const float clamped_vehicle_altitude = clamp(-_local_pos.z, 0, initial_altitude);

	// Linearly interpolate the sink rate based on the current altitude and flare parameters
	const float sink_rate = lerp(initial_altitude, initial_sink_rate, 0, _flare_sink_rate, clamped_vehicle_altitude);

	// Update altitude setpoint with the calculated sink rate
	_flare_alt_setpoint -= sink_rate * _dt;

	// Update TECS
	_tecs.set_alt_weight(2);
	_tecs.update(-_local_pos.z, _local_pos.gnd_spd, -_flare_alt_setpoint, 0, _dt);

	_position_control.roll_setpoint = 0;
	_position_control.pitch_setpoint = _tecs.get_pitch_setpoint();
	_position_control.throttle_setpoint = 0;
}

void PositionControl::tecs_update_pitch_throttle()
{
	_tecs.update(-_local_pos.z, _local_pos.gnd_spd, _tecs_setpoint.alt, _tecs_setpoint.spd, _dt);
	_position_control.pitch_setpoint = _tecs.get_pitch_setpoint();
	_position_control.throttle_setpoint = _tecs.get_throttle_setpoint();
}

void PositionControl::publish_status()
{
	_position_control.timestamp = _hal->get_time_us();
	_position_control_pub.publish(_position_control);
}

void PositionControl::reset_landing_state()
{
	_landing_state = LandingState::LOITER;
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

void PositionControl::move_point(float north, float east, float distance, float bearing_deg,
								 float *new_north, float *new_east)
{
	/*
	 * Calculate new position after moving a given distance from
	 * an original point at a given bearing
	 */
	float bearing_rad = bearing_deg * DEG_TO_RAD;
	*new_north = north + distance * cosf(bearing_rad);
	*new_east = east + distance * sinf(bearing_rad);
}
