#include "guidance.h"

Guidance::Guidance(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Guidance::init()
{

}

void Guidance::update()
{
	if (_plane->system_mode == System_mode::FLIGHT)
	{
		switch (_plane->flight_mode)
		{
		case Flight_mode::MANUAL:
			handle_manual_mode();
			break;
		case Flight_mode::AUTO:
			handle_auto_mode();
			break;
		}
	}
}

void Guidance::handle_manual_mode()
{
	update_mission();
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

// Generate position and altitude setpoint
// Detect when setpoint reached and switch to next setpoint
void Guidance::update_mission()
{
	// Determine previous waypoint
	Waypoint prev_wp;
	if (_plane->waypoint_index > 0)
	{
		prev_wp = _plane->waypoints[_plane->waypoint_index - 1];
	}
	else
	{
		// If there is no previous waypoint, use home position as prev waypoint
		prev_wp = Waypoint{
			Waypoint_type::WAYPOINT,
			_plane->home_lat,
			_plane->home_lon,
			-get_params()->takeoff_alt
		};
	}

	// Calculate previous waypoint position
	double prev_wp_north, prev_wp_east;
	lat_lon_to_meters(_plane->home_lat,
					  _plane->home_lon,
					  prev_wp.lat,
					  prev_wp.lon,
					  &prev_wp_north,
					  &prev_wp_east);

	// Calculate target waypoint position
	double tgt_wp_north, tgt_wp_east;
	Waypoint target_wp = _plane->waypoints[_plane->waypoint_index];
	lat_lon_to_meters(_plane->home_lat,
					  _plane->home_lon,
					  target_wp.lat,
					  target_wp.lon,
					  &tgt_wp_north,
					  &tgt_wp_east);

	// Calculate track heading
	float trk_hdg = atan2f(tgt_wp_east - prev_wp_east, tgt_wp_north - prev_wp_north);

	// Calculate cross track error
	float xte = cosf(trk_hdg) * (_plane->nav_pos_east - tgt_wp_east) - sinf(trk_hdg) * (_plane->nav_pos_north - tgt_wp_north);

	// Calculate heading setpoint
	_plane->guidance_hdg_setpoint = (trk_hdg + atanf(get_params()->guidance_kp * (0 - xte))) * rad_to_deg;
	if (_plane->guidance_hdg_setpoint < 0) {
		_plane->guidance_hdg_setpoint += 360.0;
	}

	// Linearly interpolate altitude setpoint
	float dist_prev_plane = sqrtf(powf(_plane->nav_pos_north - prev_wp_north, 2) + powf(_plane->nav_pos_east - prev_wp_east, 2));
	float dist_prev_tgt = sqrtf(powf(tgt_wp_north - prev_wp_north, 2) + powf(tgt_wp_east - prev_wp_east, 2));
	dist_prev_plane -= get_params()->min_dist_wp;
	if (_plane->waypoint_index != _plane->num_waypoints - 1)
	{
		dist_prev_tgt -= get_params()->min_dist_wp * 2;
	}
	else
	{
		dist_prev_tgt -= get_params()->min_dist_wp;
	}
	float progress = clamp(dist_prev_plane / dist_prev_tgt, 0, 1);
	_plane->guidance_d_setpoint = prev_wp.alt + progress * (target_wp.alt - prev_wp.alt);

	// Calculate distance to waypoint to determine if waypoint reached
	float dist_to_wp = sqrtf(powf(tgt_wp_north - _plane->nav_pos_north, 2) + powf(tgt_wp_east - _plane->nav_pos_east, 2));
	if ((dist_to_wp < get_params()->min_dist_wp) && (_plane->waypoint_index < _plane->num_waypoints - 1))
	{
		_plane->waypoint_index++;
	}
}

void Guidance::update_flare()
{
	if (!flare_initialized)
	{
		flare_start_time = _plane->time;
		flare_initialized = true;
	}

	Waypoint land_wp = _plane->waypoints[_plane->waypoint_index];
	Waypoint appr_wp = _plane->waypoints[_plane->waypoint_index - 1];
	float dist_land_appr = lat_lon_to_distance(land_wp.lat, land_wp.lon, appr_wp.lat, appr_wp.lon);
	float glideslope_angle = atan2f(land_wp.alt - appr_wp.alt, dist_land_appr);
	float time_since_flare = (_plane->time - flare_start_time) * us_to_s;
	float progress = clamp(time_since_flare / get_params()->flare_trans_sec, 0, 1);
	float initial_sink_rate = get_params()->aspd_land * sinf(glideslope_angle);
	float final_sink_rate = get_params()->flare_sink_rate;
	float sink_rate = initial_sink_rate + (final_sink_rate - initial_sink_rate) * progress;

	_plane->guidance_d_setpoint += sink_rate * _hal->get_main_dt();
}

bool Guidance::reached_wp(Waypoint wp)
{
	double wp_north, wp_east;
	lat_lon_to_meters(_plane->home_lat,
					  _plane->home_lon,
					  wp.lat,					  wp.lon,					  &wp_north,					  &wp_east);
	float dist = sqrtf(powf(_plane->nav_pos_north - wp_north, 2) +
					   powf(_plane->nav_pos_east - wp_east, 2));
	return dist < get_params()->min_dist_wp;
}

bool Guidance::reached_last_wp()
{
	bool mission_complete = false;

	double tgt_wp_north, tgt_wp_east;
	Waypoint target_wp = _plane->waypoints[_plane->waypoint_index];
	lat_lon_to_meters(_plane->home_lat, _plane->home_lon, target_wp.lat, target_wp.lon, &tgt_wp_north, &tgt_wp_east);
	float dist_to_wp = sqrtf(powf(tgt_wp_north - _plane->nav_pos_north, 2) + powf(tgt_wp_east - _plane->nav_pos_east, 2));

	// If the plane has reached the last waypoint
	if ((_plane->waypoint_index == _plane->num_waypoints - 1) && (dist_to_wp < get_params()->min_dist_wp))
	{
		mission_complete = true;
	}

	return mission_complete;
}

