#include "guidance.h"

#include <cstdio> // For testing, remove later

Guidance::Guidance(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Guidance::init()
{

}

// Generate position and altitude setpoint
// Detect when setpoint reached and switch to next setpoint
void Guidance::update_mission()
{
	double tgt_wp_north, tgt_wp_east, prev_wp_north, prev_wp_east;

	// Determine target waypoint
	Waypoint target_wp = _plane->waypoints[_plane->waypoint_index];
	lat_lon_to_meters(_plane->center_lat, _plane->center_lon, target_wp.lat, target_wp.lon, &tgt_wp_north, &tgt_wp_east);
	_plane->guidance_d_setpoint = target_wp.alt;

	// Determine previous waypoint
	Waypoint prev_wp;
	if (_plane->waypoint_index > 0)
	{
		prev_wp = _plane->waypoints[_plane->waypoint_index - 1];
	}
	else
	{
		prev_wp = Waypoint{_plane->center_lat, _plane->center_lon, 0};
	}
	lat_lon_to_meters(_plane->center_lat, _plane->center_lon, prev_wp.lat, prev_wp.lon, &prev_wp_north, &prev_wp_east);

	// Calculate track heading
	float trk_hdg = atan2f(tgt_wp_east - prev_wp_east, tgt_wp_north - prev_wp_north);

	// Calculate cross track error
	float xte = cosf(trk_hdg) * (_plane->nav_pos_east - tgt_wp_east) - sinf(trk_hdg) * (_plane->nav_pos_north - tgt_wp_north);

	// Calculate heading setpoint
	_plane->guidance_hdg_setpoint = trk_hdg * 180.0f / M_PI + clamp(kP * (0 - xte), -90, 90);
	if (_plane->guidance_hdg_setpoint < 0) {
		_plane->guidance_hdg_setpoint += 360.0;
	}

	// Calculate distance to waypoint to determine if waypoint reached
	float err_n = tgt_wp_north - _plane->nav_pos_north;
	float err_e = tgt_wp_east - _plane->nav_pos_east;
	float dist_to_wp = sqrtf(powf(err_n, 2) + powf(err_e, 2));
	if ((dist_to_wp < MIN_DIST_WP) && (_plane->waypoint_index < _plane->num_waypoints - 1))
	{
		_plane->waypoint_index++;
	}
}

void Guidance::update_landing()
{
	double land_north, land_east;
	lat_lon_to_meters(_plane->center_lat, _plane->center_lon, _plane->land_lat, _plane->land_lon, &land_north, &land_east);

	float dist_to_land = sqrtf(powf(_plane->nav_pos_north - land_north, 2) +
							   powf(_plane->nav_pos_east - land_east, 2));

	// Follow glideslope angle
	_plane->guidance_d_setpoint = -dist_to_land * sinf(LAND_GS_DEG * M_PI / 180.0f);

	// Set track heading to runway heading
	float trk_hdg = _plane->land_hdg * M_PI / 180.0f;

	// Calculate cross track error
	float xte = cosf(trk_hdg) * (_plane->nav_pos_east - land_east) -
				sinf(trk_hdg) * (_plane->nav_pos_north - land_north);

	// Calculate heading setpoint
	_plane->guidance_hdg_setpoint = trk_hdg * 180.0f / M_PI + clamp(kP * (0 - xte), -90, 90);
	if (_plane->guidance_hdg_setpoint < 0) {
		_plane->guidance_hdg_setpoint += 360.0;
	}
}

void Guidance::update_flare()
{
	// Decrease altitude setpoint at a rate of FLARE_SINK_RATE
	_plane->guidance_d_setpoint = _plane->flare_alt + FLARE_SINK_RATE * (_plane->time - _plane->flare_start_time) / 1000000.0f;
}

