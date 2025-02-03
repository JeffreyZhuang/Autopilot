#include <autopilot.h>

// Use the three switch for direct, stab, and auto
// Then one switch that can override and force manual.

// I don't need an enum for manual

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal, Plane* plane): _ahrs(hal, plane, hal->main_dt),
									   	   	  _navigation(hal, plane, hal->main_dt),
											  _control(hal, plane, hal->main_dt),
											  _guidance(hal, plane),
											  _telem(hal, plane)
{
	_hal = hal;
	_plane = plane;
	_instance = this;
}

void Autopilot::setup()
{
	init_state();

	_hal->init();

	_ahrs.setup();
	_guidance.init();

	// Start tasks
	_hal->set_main_task(&Autopilot::static_main_task);
	_hal->set_background_task(&Autopilot::static_logger_task);
}

void Autopilot::main_task()
{
	update_time();
	_hal->read_sensors();

	evaluate_system_mode();

	_telem.update();
	_hal->write_storage_buffer();
}

void Autopilot::logger_task()
{
	_hal->flush_storage_buffer();
}

/**
 * System Modes
 */
void Autopilot::evaluate_system_mode()
{
	_plane->mode_id = static_cast<uint8_t>(_plane->systemMode);

	switch (_plane->systemMode)
	{
	case SystemMode::BOOT:
		boot();
		break;
	case SystemMode::FLIGHT:
		flight();
		break;
	}
}

void Autopilot::boot()
{
	_ahrs.set_initial_state();

	// Zero barometer
	_plane->baro_offset = _plane->baro_alt;

	// Set home position to first GPS fix
	bool gnss_locked = (_plane->gnss_sats > 5) && (fabs(_plane->gnss_lat) > 0) && (fabs(_plane->gnss_lat) > 0);
	if (gnss_locked)
	{
		_plane->gnss_center_lat = _plane->gnss_lat;
		_plane->gnss_center_lon = _plane->gnss_lon;

		_navigation.execute();
	}

//	bool transmitter_safe = (_plane->rc_throttle < THR_DEADZONE) && (_plane->manual_sw == false);
	bool transmitter_safe = (_plane->rc_throttle < 0.1) && (_plane->manual_sw == false) && (_plane->mode_sw == false);

	if (gnss_locked && transmitter_safe)
	{
		_plane->systemMode = SystemMode::FLIGHT;
	}
}

void Autopilot::flight()
{
	_ahrs.update();
	_navigation.execute();

	if (_plane->manual_sw)
	{
		if (_plane->mode_sw)
		{
			evaluate_auto_mode();
		}
		else
		{
			_plane->manualMode = ManualMode::STABILIZED;
			evaluate_manual_mode();
		}
	}
	else
	{
		_plane->manualMode = ManualMode::MANUAL;
		evaluate_manual_mode();
	}
}

/**
 * Auto Modes
 */
void Autopilot::evaluate_auto_mode()
{
	_plane->mode_id = static_cast<uint8_t>(_plane->autoMode);

	switch (_plane->autoMode)
	{
	case AutoMode::READY:
		ready();
		break;
	case AutoMode::TAKEOFF:
		takeoff();
		break;
	case AutoMode::MISSION:
		mission();
		break;
	case AutoMode::LAND:
		land();
		break;
	case AutoMode::FLARE:
		flare();
		break;
	case AutoMode::SAFE:
		safe();
		break;
	}
}

void Autopilot::ready()
{
	if (_plane->rc_throttle > 0.5)
	{
		_plane->autoMode = AutoMode::TAKEOFF;
	}
}

void Autopilot::takeoff()
{
	_control.update_takeoff();

	if (-_plane->nav_pos_down > TAKEOFF_ALT)
	{
		_plane->autoMode = AutoMode::MISSION;
	}
}

void Autopilot::mission()
{
	_guidance.update_mission();

	_control.update_mission();

	if (_plane->waypoint_index == _plane->num_waypoints)
	{
		// Switch to land
	}
}

void Autopilot::land()
{
	_guidance.update_landing();

	_control.update_land();

	if (-_plane->nav_pos_down < LAND_FLARE_ALT)
	{
		_plane->autoMode = AutoMode::FLARE;
	}
}

void Autopilot::flare()
{

}

void Autopilot::safe()
{

}

/**
 * Manual Modes
 */
void Autopilot::evaluate_manual_mode()
{
	_plane->mode_id = static_cast<uint8_t>(_plane->manualMode);

	_guidance.update_mission();

	switch (_plane->manualMode)
	{
	case ManualMode::MANUAL:
		_control.update_manual();
		break;
	case ManualMode::STABILIZED:
		_control.update_stabilized();
		break;
	}
}

/**
 * Helper functions
 */
void Autopilot::init_state()
{
	_plane->systemMode = SystemMode::BOOT;
	_plane->manualMode = ManualMode::MANUAL;
	_plane->autoMode = AutoMode::READY;
}

void Autopilot::update_time()
{
	uint64_t time = _hal->get_time_us();
	_plane->loop_execution_time = time - _plane->time;
	_plane->time = time;
	_plane->loop_iteration++;
}
