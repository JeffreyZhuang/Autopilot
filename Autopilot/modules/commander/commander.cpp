#include "modules/commander/commander.h"

Commander::Commander(HAL* hal, DataBus* data_bus)
	: Module(hal, data_bus),
	  _ahrs_sub(data_bus->ahrs_node),
	  _local_pos_sub(data_bus->local_position_node),
	  _rc_sub(data_bus->rc_node),
	  _waypoint_sub(data_bus->waypoint_node),
	  _modes_pub(data_bus->modes_node)
{
	_modes_data.system_mode = System_mode::LOAD_PARAMS;
	_modes_data.flight_mode = Flight_mode::MANUAL;
	_modes_data.manual_mode = Manual_mode::DIRECT;
	_modes_data.auto_mode = Auto_mode::TAKEOFF;
	_modes_pub.publish(_modes_data);
}

void Commander::update_parameters()
{
	param_get(TKO_ALT, &_takeoff_alt);
	param_get(LND_FL_ALT, &_flare_alt);
}

void Commander::poll_vehicle_data()
{
	_local_pos = _local_pos_sub.get();
	_ahrs_data = _ahrs_sub.get();
	_rc_data = _rc_sub.get();
	_waypoint = _waypoint_sub.get();
}

void Commander::update()
{
	update_parameters();
	poll_vehicle_data();

	switch (_modes_data.system_mode)
	{
	case System_mode::LOAD_PARAMS:
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

void Commander::update_config()
{
	if (param_all_set())
	{
		_modes_data.system_mode = System_mode::STARTUP;
	}
}

void Commander::update_startup()
{
	bool transmitter_safe = _rc_data.thr_norm == 0 &&
							!_rc_data.man_sw &&
							!_rc_data.mod_sw;

	if (_ahrs_data.converged && _local_pos.converged &&
		_rc_data.tx_conn && transmitter_safe &&
		mission_get().mission_type != MISSION_EMPTY)
	{
		_modes_data.system_mode = System_mode::FLIGHT;
	}
}

void Commander::handle_flight_mode()
{
	handle_switches();
	handle_auto_mode();
}

void Commander::handle_switches()
{
	// Manual switch overrides mode switch and toggles between manual
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

void Commander::handle_auto_mode()
{
	switch (_modes_data.auto_mode)
	{
	case Auto_mode::TAKEOFF:
		update_takeoff();
		break;
	case Auto_mode::MISSION:
		break;
	}
}

void Commander::update_takeoff()
{
	if (-_local_pos.z > _takeoff_alt)
	{
		_modes_data.auto_mode = Auto_mode::MISSION;
	}
}
