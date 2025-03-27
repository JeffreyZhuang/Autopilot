#include "modules/commander/commander.h"

Commander::Commander(HAL* hal)
	: Module(hal),
	  _ahrs_sub(Data_bus::get_instance().ahrs_data),
	  _pos_est_sub(Data_bus::get_instance().pos_est_data),
	  _rc_sub(Data_bus::get_instance().rc_data),
	  _telem_sub(Data_bus::get_instance().telem_data),
	  _navigator_sub(Data_bus::get_instance().navigator_data),
	  _modes_pub(Data_bus::get_instance().modes_data)
{
	_modes_data.system_mode = System_mode::CONFIG;
	_modes_data.flight_mode = Flight_mode::MANUAL;
	_modes_data.manual_mode = Manual_mode::DIRECT;
	_modes_data.auto_mode = Auto_mode::TAKEOFF;
	_modes_pub.publish(_modes_data);
}

void Commander::update()
{
	_pos_est_data = _pos_est_sub.get();
	_ahrs_data = _ahrs_sub.get();
	_rc_data = _rc_sub.get();
	_telem_data = _telem_sub.get();
	_navigator_data = _navigator_sub.get();

	switch (_modes_data.system_mode)
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

	_modes_pub.publish(_modes_data);
}

void Commander::handle_flight_mode()
{
	handle_switches();

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

void Commander::handle_manual_mode()
{
	switch (_modes_data.manual_mode)
	{
	case Manual_mode::DIRECT:
		break;
	case Manual_mode::STABILIZED:
		break;
	}
}

void Commander::handle_auto_mode()
{
	switch (_modes_data.auto_mode)
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
	if (_rc_data.man_sw)
	{
		// Mode switch toggles between stabilized and auto
		if (_rc_data.mod_sw)
		{
			_modes_data.flight_mode = Flight_mode::AUTO;
		}
		else
		{
			_modes_data.manual_mode = Manual_mode::STABILIZED;
			_modes_data.flight_mode = Flight_mode::MANUAL;
		}
	}
	else
	{
		_modes_data.manual_mode = Manual_mode::DIRECT;
		_modes_data.flight_mode = Flight_mode::MANUAL;
	}
}

void Commander::update_config()
{
	if (are_params_set() && _telem_data.waypoints_loaded)
	{
		if (get_params()->hitl.enable)
		{
			_hal->enable_hitl();
		}

		_modes_data.system_mode = System_mode::STARTUP;
	}
}

void Commander::update_startup()
{
	bool transmitter_safe = _rc_data.thr_norm == 0 &&
							!_rc_data.man_sw &&
							!_rc_data.mod_sw;

	if (_ahrs_data.converged &&
		_pos_est_data.converged &&
		_rc_data.tx_conn &&
		transmitter_safe)
	{
		_modes_data.system_mode = System_mode::FLIGHT;
	}
}

void Commander::update_takeoff()
{
	if (-_pos_est_data.pos_d > get_params()->takeoff.alt)
	{
		_modes_data.auto_mode = Auto_mode::MISSION;
	}
}

void Commander::update_mission()
{
	if (_navigator_data.waypoint_index == _telem_data.num_waypoints - 1)
	{
		_modes_data.auto_mode = Auto_mode::LAND;
	}
}

void Commander::update_land()
{
	if (-_pos_est_data.pos_d < get_params()->landing.flare_alt)
	{
		_modes_data.auto_mode = Auto_mode::FLARE;
	}
}

void Commander::update_flare()
{
	// Detect touchdown when speed is below TOUCHDOWN_SPD_THR
	if (_pos_est_data.gnd_spd < get_params()->landing.touchdown_speed)
	{
		_modes_data.auto_mode = Auto_mode::TOUCHDOWN;
	}
}

void Commander::update_touchdown()
{

}
