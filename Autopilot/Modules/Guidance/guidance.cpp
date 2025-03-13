#include "guidance.h"

Guidance::Guidance(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Guidance::update()
{
	if (_plane->system_mode == System_mode::FLIGHT)
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

void Guidance::handle_auto_mode()
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

// Generate position and altitude setpoints
// Detect when waypoint reached and switch to next waypoint
void Guidance::update_mission()
{
	// Determine previous waypoint or fallback to home position
	Waypoint prev_wp = (_plane->waypoint_index > 0) ?
		_plane->waypoints[_plane->waypoint_index - 1] :
		Waypoint{_plane->home_lat, _plane->home_lon, -get_params()->takeoff_alt};

	// Get target waypoint
	Waypoint target_wp = _plane->waypoints[_plane->waypoint_index];

	// Convert waypoints to north east coordinates
	double prev_north, prev_east, tgt_north, tgt_east;
	lat_lon_to_meters(_plane->home_lat, _plane->home_lon, prev_wp.lat, prev_wp.lon, &prev_north, &prev_east);
	lat_lon_to_meters(_plane->home_lat, _plane->home_lon, target_wp.lat, target_wp.lon, &tgt_north, &tgt_east);

	// Calculate track heading (bearing from previous to target waypoint)
	float trk_hdg = atan2f(tgt_east - prev_east, tgt_north - prev_north);

	// Compute cross-track error (perpendicular distance from aircraft to path)
	float rel_east = _plane->nav_pos_east - tgt_east;
	float rel_north = _plane->nav_pos_north - tgt_north;
	float xte = cosf(trk_hdg) * rel_east - sinf(trk_hdg) * rel_north;

	// Calculate heading setpoint using proportional guidance law
	_plane->guidance_hdg_setpoint = (trk_hdg + atanf(get_params()->guidance_kp * -xte)) * rad_to_deg;
	if (_plane->guidance_hdg_setpoint < 0) {
		_plane->guidance_hdg_setpoint += 360.0; // Normalize to [0, 360] range
	}

	// Compute along-track distance (projected aircraft position onto path)
	float vec_north = tgt_north - prev_north;
	float vec_east = tgt_east - prev_east;
	float vec_norm = sqrtf(vec_north * vec_north + vec_east * vec_east);
	float proj_factor = ((_plane->nav_pos_north - prev_north) * vec_north +
						 (_plane->nav_pos_east - prev_east) * vec_east) / (vec_norm * vec_norm);
	float along_track_dist = proj_factor * vec_norm;

	// Interpolate altitude setpoints
	if (along_track_dist > 0 || _plane->waypoint_index == 0)
	{
		// Interpolate between previous and target waypoint
		_plane->guidance_d_setpoint = lerp(0, prev_wp.alt, vec_norm, target_wp.alt, along_track_dist);
	}
	else
	{
		// Use the waypoint before the previous one, or fallback to home
		Waypoint prev_prev_wp = (_plane->waypoint_index > 1) ?
			_plane->waypoints[_plane->waypoint_index - 2] :
			Waypoint{_plane->home_lat, _plane->home_lon, -get_params()->takeoff_alt};

		double prev_prev_north, prev_prev_east;
		lat_lon_to_meters(_plane->home_lat, _plane->home_lon, prev_prev_wp.lat, prev_prev_wp.lon, &prev_prev_north, &prev_prev_east);

		// Compute along-track distance (projected aircraft position onto path)
		vec_north = prev_north - prev_prev_north;
		vec_east = prev_east - prev_prev_east;
		vec_norm = sqrtf(vec_north * vec_north + vec_east * vec_east);
		proj_factor = ((_plane->nav_pos_north - prev_prev_north) * vec_north +
							 (_plane->nav_pos_east - prev_prev_east) * vec_east) / (vec_norm * vec_norm);
		along_track_dist = proj_factor * vec_norm;

		// Interpolate between previous waypoint and the one before that
		_plane->guidance_d_setpoint = lerp(0, prev_prev_wp.alt, vec_norm, prev_wp.alt, along_track_dist);
	}

	// Check distance to waypoint to determine if waypoint reached
	float dist_to_wp = sqrtf(rel_north*rel_north + rel_east*rel_east);
	if ((dist_to_wp < get_params()->min_dist_wp) && (_plane->waypoint_index < _plane->num_waypoints - 1))
	{
		_plane->waypoint_index++; // Move to next waypoint
	}
}

void Guidance::update_flare()
{
	// Get the landing waypoint and the approach waypoint
	const Waypoint& land_wp = _plane->waypoints[_plane->waypoint_index];
	const Waypoint& appr_wp = _plane->waypoints[_plane->waypoint_index - 1];

	// Calculate the horizontal distance between the landing and approach waypoints
	float dist_land_appr = lat_lon_to_distance(
		land_wp.lat, land_wp.lon,
		appr_wp.lat, appr_wp.lon
	);

	// Calculate the glideslope angle based on the altitude difference and horizontal distance
	float glideslope_angle = atan2f(
		land_wp.alt - appr_wp.alt,
		dist_land_appr
	);

	// Calculate the initial sink rate during flare based on the glideslope angle and landing airspeed
	float flare_initial_sink_rate = get_params()->aspd_land * sinf(glideslope_angle);

	// Linearly interpolate the sink rate based on the current altitude and flare parameters
	float sink_rate = lerp(get_params()->flare_alt, flare_initial_sink_rate, 0, get_params()->flare_sink_rate, clamp(-_plane->nav_pos_down, 0, get_params()->flare_alt));

	// Update the guidance setpoint with the calculated sink rate
	_plane->guidance_d_setpoint += sink_rate * _hal->get_main_dt();
}
