#include "guidance.h"

Guidance::Guidance(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Guidance::update()
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
	// Determine previous waypoint
	Waypoint prev_wp = _plane->waypoints[_plane->waypoint_index - 1];

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
	_plane->guidance_hdg_setpoint = (trk_hdg + atanf(get_params()->guidance_kp * -xte)) * RAD_TO_DEG;
	if (_plane->guidance_hdg_setpoint < 0) {
		_plane->guidance_hdg_setpoint += 360.0; // Normalize to [0, 360] range
	}

	// Determine altitude setpoint
	if (_plane->waypoint_index == 1)
	{
		_plane->guidance_d_setpoint = _plane->waypoints[1].alt;
	}
	else
	{
		float along_track_dist = compute_along_track_distance(
			prev_north,
			prev_east,
			tgt_north,
			tgt_east,
			_plane->nav_pos_north,
			_plane->nav_pos_east
		);

		// Interpolate altitude setpoints
		if (along_track_dist > 0)
		{
			// Interpolate between previous and target waypoint
			_plane->guidance_d_setpoint = lerp(
				0,
				prev_wp.alt,
				distance(prev_north, prev_east, tgt_north, tgt_east),
				target_wp.alt,
				along_track_dist
			);
		}
		else if (_plane->waypoint_index == 2)
		{
			_plane->guidance_d_setpoint = _plane->waypoints[1].alt;
		}
		else
		{
			// Use the waypoint before the previous one
			Waypoint prev_prev_wp = _plane->waypoints[_plane->waypoint_index - 2];

			double prev_prev_north, prev_prev_east;
			lat_lon_to_meters(_plane->home_lat, _plane->home_lon, prev_prev_wp.lat, prev_prev_wp.lon, &prev_prev_north, &prev_prev_east);

			// Compute along-track distance (projected aircraft position onto path)
			float along_track_dist_prev = compute_along_track_distance(prev_prev_north, prev_prev_east, prev_north, prev_east,
																				   _plane->nav_pos_north, _plane->nav_pos_east);

			// Interpolate between previous waypoint and the one before that
			_plane->guidance_d_setpoint = lerp(
				0,
				prev_prev_wp.alt,
				distance(prev_prev_north, prev_prev_east, prev_north, prev_east),
				prev_wp.alt,
				along_track_dist_prev
			);
		}
	}

	// Check distance to waypoint to determine if waypoint reached
	float dist_to_wp = sqrtf(rel_north*rel_north + rel_east*rel_east);
	if ((dist_to_wp < get_params()->min_dist_wp) &&
		(_plane->waypoint_index < _plane->num_waypoints - 1))
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
	float sink_rate = lerp(get_params()->flare_alt,
						   flare_initial_sink_rate,
						   0,
						   get_params()->flare_sink_rate,
						   clamp(-_plane->nav_pos_down, 0, get_params()->flare_alt));

	// Make sure sink rate decreases during flare, not increases
	if (sink_rate > flare_initial_sink_rate)
	{
		sink_rate = flare_initial_sink_rate;
	}

	// Update the guidance setpoint with the calculated sink rate
	_plane->guidance_d_setpoint += sink_rate * _plane->dt_s;
}

// Helper function to compute along-track distance (projected aircraft position onto path)
float Guidance::compute_along_track_distance(float start_n, float start_e, float end_n, float end_e,
											 float pos_n, float pos_e)
{
	float vec_north = end_n - start_n;
	float vec_east = end_e - start_e;
	float vec_norm = sqrtf(vec_north * vec_north + vec_east * vec_east);
	float proj_factor = ((pos_n - start_n) * vec_north + (pos_e - start_e) * vec_east) / (vec_norm * vec_norm);
	return proj_factor * vec_norm;
}

// Helper function to compute Euclidean distance
float Guidance::distance(float n1, float e1, float n2, float e2)
{
	float dn = n2 - n1;
	float de = e2 - e1;
	return sqrtf(dn * dn + de * de);
}
