#include "modules/commander/commander.h"

Commander::Commander(HAL* hal, Plane* plane) : Module(hal, plane)
{
	_plane->system_mode = Plane::System_mode::CONFIG;
	_plane->flight_mode = Plane::Flight_mode::MANUAL;
	_plane->manual_mode = Plane::Manual_mode::DIRECT;
	_plane->auto_mode = Plane::Auto_mode::TAKEOFF;
}

void Commander::update()
{
	_pos_est_data = _plane->get_pos_est_data(_pos_est_handle);
	_ahrs_data = _plane->get_ahrs_data(_ahrs_handle);

	switch (_plane->system_mode)
	{
	case Plane::System_mode::CONFIG:
		update_config();
		break;
	case Plane::System_mode::STARTUP:
		update_startup();
		break;
	case Plane::System_mode::FLIGHT:
		handle_flight_mode();
		break;
	}
}

void Commander::handle_flight_mode()
{
	handle_switches();

	switch (_plane->flight_mode)
	{
	case Plane::Flight_mode::MANUAL:
		handle_manual_mode();
		break;
	case Plane::Flight_mode::AUTO:
		handle_auto_mode();
		break;
	}
}

void Commander::handle_manual_mode()
{
	switch (_plane->manual_mode)
	{
	case Plane::Manual_mode::DIRECT:
		break;
	case Plane::Manual_mode::STABILIZED:
		break;
	}
}

void Commander::handle_auto_mode()
{
	switch (_plane->auto_mode)
	{
	case Plane::Auto_mode::TAKEOFF:
		update_takeoff();
		break;
	case Plane::Auto_mode::MISSION:
		update_mission();
		break;
	case Plane::Auto_mode::LAND:
		update_land();
		break;
	case Plane::Auto_mode::FLARE:
		update_flare();
		break;
	case Plane::Auto_mode::TOUCHDOWN:
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
			_plane->flight_mode = Plane::Flight_mode::AUTO;
		}
		else
		{
			_plane->manual_mode = Plane::Manual_mode::STABILIZED;
			_plane->flight_mode = Plane::Flight_mode::MANUAL;
		}
	}
	else
	{
		_plane->manual_mode = Plane::Manual_mode::DIRECT;
		_plane->flight_mode = Plane::Flight_mode::MANUAL;
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

		_plane->system_mode = Plane::System_mode::STARTUP;
	}
}

void Commander::update_startup()
{
	bool transmitter_safe = _plane->rc_thr_norm == 0 &&
							!_plane->rc_man_sw &&
							!_plane->rc_mod_sw;

	if (_ahrs_data.converged &&
		_pos_est_data.converged &&
		_plane->tx_connected &&
		transmitter_safe)
	{
		_plane->system_mode = Plane::System_mode::FLIGHT;
	}
}

void Commander::update_takeoff()
{
	if (-_pos_est_data.pos_d > get_params()->takeoff.alt)
	{
		_plane->auto_mode = Plane::Auto_mode::MISSION;
	}
}

void Commander::update_mission()
{
	if (_plane->waypoint_index == _plane->num_waypoints - 1)
	{
		_plane->auto_mode = Plane::Auto_mode::LAND;
	}
}

void Commander::update_land()
{
	if (-_pos_est_data.pos_d < get_params()->landing.flare_alt)
	{
		_plane->auto_mode = Plane::Auto_mode::FLARE;
	}
}

void Commander::update_flare()
{
	// Detect touchdown when speed is below TOUCHDOWN_SPD_THR
	if (_pos_est_data.gnd_spd < get_params()->landing.touchdown_speed)
	{
		_plane->auto_mode = Plane::Auto_mode::TOUCHDOWN;
	}
}

void Commander::update_touchdown()
{

}
