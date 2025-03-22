#include "commander.h"

Commander::Commander(HAL* hal, Plane* plane) : Module(hal, plane)
{
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
	if (_plane->rc_man_sw)
	{
		// Mode switch toggles between stabilized and auto
		if (_plane->rc_mod_sw)
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
	if (are_params_set() && _plane->waypoints_loaded)
	{
		if (get_params()->hitl.enable)
		{
			_hal->enable_hitl();
		}

		_plane->system_mode = System_mode::STARTUP;
	}
}

void Commander::update_startup()
{
	bool transmitter_safe = _plane->rc_thr_norm == 0 &&
							!_plane->rc_man_sw &&
							!_plane->rc_mod_sw;
	if (_plane->ahrs_converged &&
		_plane->nav_converged &&
		_plane->tx_connected &&
		transmitter_safe)
	{
		_plane->system_mode = System_mode::FLIGHT;
	}
}

void Commander::update_takeoff()
{
	if (-_plane->nav_pos_down > get_params()->takeoff.alt)
	{
		_plane->auto_mode = Auto_mode::MISSION;
	}
}

void Commander::update_mission()
{
	if (_plane->waypoint_index == _plane->num_waypoints - 1)
	{
		_plane->auto_mode = Auto_mode::LAND;
	}
}

void Commander::update_land()
{
	if (-_plane->nav_pos_down < get_params()->landing.flare_alt)
	{
		_plane->auto_mode = Auto_mode::FLARE;
	}
}

void Commander::update_flare()
{
	// Detect touchdown when speed is below TOUCHDOWN_SPD_THR
	if (_plane->nav_gnd_spd < get_params()->landing.touchdown_speed)
	{
		_plane->auto_mode = Auto_mode::TOUCHDOWN;
	}
}

void Commander::update_touchdown()
{

}
