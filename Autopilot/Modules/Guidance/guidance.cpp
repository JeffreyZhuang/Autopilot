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
		update_mission();
		break;
	case Auto_mode::LAND:
		update_landing();
		break;
	case Auto_mode::FLARE:
		update_flare();
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
		prev_wp = Waypoint{Waypoint_type::WAYPOINT, _plane->home_lat, _plane->home_lon, get_params()->takeoff_alt};
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
	_plane->guidance_hdg_setpoint = trk_hdg * rad_to_deg + clamp(kP * (0 - xte), -90, 90);
	if (_plane->guidance_hdg_setpoint < 0) {
		_plane->guidance_hdg_setpoint += 360.0;
	}

	// Linearly interpolate altitude setpoint
	float dist_prev_plane = sqrtf(powf(_plane->nav_pos_north - prev_wp_north, 2) + powf(_plane->nav_pos_east - prev_wp_east, 2));
	float dist_prev_tgt = sqrtf(powf(tgt_wp_north - prev_wp_north, 2) + powf(tgt_wp_east - prev_wp_east, 2));
	float progress = dist_prev_plane / dist_prev_tgt;
	if (progress > 1)
	{
		progress = 1;
	}
	_plane->guidance_d_setpoint = prev_wp.alt + progress * (target_wp.alt - prev_wp.alt);

	// Calculate distance to waypoint to determine if waypoint reached
	float dist_to_wp = sqrtf(powf(tgt_wp_north - _plane->nav_pos_north, 2) + powf(tgt_wp_east - _plane->nav_pos_east, 2));
	if ((dist_to_wp < get_params()->min_dist_wp) && (_plane->waypoint_index < _plane->num_waypoints - 1))
	{
		_plane->waypoint_index++;
	}
}

void Guidance::update_landing()
{
	// Calculate previous waypoint position
	Waypoint prev_wp = _plane->waypoints[_plane->waypoint_index - 1];
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
	_plane->guidance_hdg_setpoint = trk_hdg * rad_to_deg + clamp(kP * (0 - xte), -90, 90);
	if (_plane->guidance_hdg_setpoint < 0) {
		_plane->guidance_hdg_setpoint += 360.0;
	}

	// Follow glideslope angle
	float dist_to_land = sqrtf(powf(tgt_wp_north - _plane->nav_pos_north, 2) + powf(tgt_wp_east - _plane->nav_pos_east, 2));
	_plane->guidance_d_setpoint = -dist_to_land * sinf(get_params()->land_gs_deg * deg_to_rad);

	// Prevent needlessly climbing during approach
	// Don't target an altitude higher than the final waypoint
	if (_plane->guidance_d_setpoint < _plane->waypoints[-1].alt)
	{
		_plane->guidance_d_setpoint = _plane->waypoints[-1].alt;
	}
}

void Guidance::update_flare()
{
	// Glideslope altitude divided by horizontal distance
	float initial_slope = tanf(get_params()->land_gs_deg * deg_to_rad);

	// Initial Slope = Altitude / Horizontal distance from landing point
	// Horizontal distance = Altitude / Initial slope
	float initial_dist = get_params()->land_flare_alt / initial_slope;











	float time_since_flare_s = (_plane->time - _plane->flare_start_time) * us_to_s;

	// Estimate initial sink rate from glideslope
	float flare_initial_sinkrate = get_params()->aspd_land * sinf(get_params()->land_gs_deg * deg_to_rad);

	// Calculate sink acceleration
	// Acceleration = (Initial Sink Rate - Final Sink Rate) / Flare Transition Time
	float sink_accel = (flare_initial_sinkrate - get_params()->flare_sink_rate) / get_params()->flare_trans_sec;
	if (sink_accel < 0)
	{
		sink_accel = 0;
	}

	// Decrease sink rate over time based on sink_accel
	float flare_sink_rate = flare_initial_sinkrate - sink_accel * time_since_flare_s;
	if (flare_sink_rate > get_params()->flare_sink_rate)
	{
		flare_sink_rate = get_params()->flare_sink_rate; // Set minimum sink rate to FLARE_SINK_RATE
	}

	// Decrease altitude setpoint at a rate of _flare_sink_rate
	// THis is wrong because not avg!
	_plane->guidance_d_setpoint = _plane->flare_alt + flare_sink_rate * time_since_flare_s;
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

