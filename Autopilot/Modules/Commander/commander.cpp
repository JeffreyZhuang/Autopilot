#include "commander.h"

Commander::Commander(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Commander::update()
{
	switch (_plane->system_mode)
	{
	case System_mode::CONFIG:
		update_config();
		break;
	case System_mode::STARTUP:
		update_startup();
		break;
	case System_mode::FLIGHT:
		handle_flight_mode();
		break;
	}
}

void Commander::handle_flight_mode()
{
	handle_switches();

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

void Commander::handle_manual_mode()
{
	switch (_plane->manual_mode)
	{
	case Manual_mode::DIRECT:
		break;
	case Manual_mode::STABILIZED:
		break;
	}
}

void Commander::handle_auto_mode()
{
	switch (_plane->auto_mode)
	{
	case Auto_mode::TAKEOFF:
		update_takeoff();
		break;
	case Auto_mode::MISSION:
		update_mission();
		break;
	case Auto_mode::LAND:
		update_land();
		break;
	case Auto_mode::FLARE:
		update_flare();
		break;
	case Auto_mode::TOUCHDOWN:
		update_touchdown();
		break;
	}
}

void Commander::handle_switches()
{
	// Manual switch overrides Mode switch and toggles between manual
	if (_plane->manual_sw)
	{
		// Mode switch toggles between stabilized and auto
		if (_plane->mode_sw)
		{
			_plane->flight_mode = Flight_mode::AUTO;
		}
		else
		{
			_plane->manual_mode = Manual_mode::STABILIZED;
			_plane->flight_mode = Flight_mode::MANUAL;
		}
	}
	else
	{
		_plane->manual_mode = Manual_mode::DIRECT;
		_plane->flight_mode = Flight_mode::MANUAL;
	}
}

void Commander::update_config()
{
	if (get_params()->set)
	{
		_plane->system_mode = System_mode::STARTUP;
	}
}

void Commander::update_startup()
{
	bool waypoints_loaded = _plane->num_waypoints > 0;
	bool transmitter_safe = _plane->rc_in_norm[get_params()->throttle_ch] == 0 &&
							!_plane->manual_sw && !_plane->mode_sw;
	if (_plane->ahrs_converged &&
		_plane->nav_converged &&
		transmitter_safe &&
		waypoints_loaded &&
		_plane->tx_connected)
	{
		_plane->system_mode = System_mode::FLIGHT;
	}
}

void Commander::update_takeoff()
{
	if (-_plane->nav_pos_down > get_params()->takeoff_alt)
	{
		_plane->auto_mode = Auto_mode::MISSION;
	}
}

void Commander::update_mission()
{
	if (_plane->waypoints[_plane->waypoint_index].type == Waypoint_type::LAND)
	{
		_plane->auto_mode = Auto_mode::LAND;
	}
}

void Commander::update_land()
{
	if (_plane->rangefinder_dist < get_params()->land_flare_alt)
	{
		_plane->auto_mode = Auto_mode::FLARE;
		_plane->flare_alt = _plane->nav_pos_down;
		_plane->flare_start_time = _plane->time;
	}
}

void Commander::update_flare()
{
	// Detect touchdown when speed is below TOUCHDOWN_SPD_THR
	if (_plane->nav_airspeed < get_params()->touchdown_aspd_thresh)
	{
		_plane->auto_mode = Auto_mode::TOUCHDOWN;
	}
}

void Commander::update_touchdown()
{

}
