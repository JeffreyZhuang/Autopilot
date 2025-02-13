#include "autopilot.h"

// TODO: Use COBS and start byte so a single missed byte doesn't corrupt the entire file.

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal, Plane* plane): _ahrs(hal, plane, hal->get_main_dt()),
									   	   	  _navigation(hal, plane, hal->get_main_dt()),
											  _control(hal, plane, hal->get_main_dt()),
											  _guidance(hal, plane),
											  _telem(hal, plane),
											  _storage(plane, hal)
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
	_storage.write();
}

void Autopilot::logger_task()
{
	_storage.flush();
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

// Set initial state only once, and then update AHRS normally so you have time for filter to converge
// Then check for filter converge before moving out of boot
void Autopilot::boot()
{
	if (_ahrs.set_initial_state())
	{
		_ahrs.update();
	}

	// Set home position to first GPS fix
	if (_navigation.set_home())
	{
		_navigation.execute();
		_navigation.debug_serial();
	}

	bool waypoints_loaded = _plane->num_waypoints > 0;

	bool transmitter_safe = (_plane->rc_throttle < 0.1) && (_plane->manual_sw == false) && (_plane->mode_sw == false);

	if (_plane->gps_fix && transmitter_safe && waypoints_loaded)
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
	case AutoMode::TOUCHDOWN:
		touchdown();
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

	if (_guidance.reached_last_wp())
	{
		_plane->autoMode = AutoMode::LAND;
	}
}

// Glide path to runway
void Autopilot::land()
{
	_guidance.update_landing();
	_control.update_land();

	if (_plane->rangefinder_dist < LAND_FLARE_ALT)
	{
		_plane->autoMode = AutoMode::FLARE;
		_plane->flare_alt = _plane->nav_pos_down;
		_plane->flare_start_time = _plane->time;
	}
}

// Lower descent rate below altitude threshold
void Autopilot::flare()
{
	_guidance.update_flare();
	_control.update_flare();

	if (_plane->time - _plane->flare_start_time > 10000000)
	{
		_plane->autoMode = AutoMode::TOUCHDOWN;
	}
}

void Autopilot::touchdown()
{
	_hal->set_rudder(0);
	_hal->set_elevator(0);
	_hal->set_throttle(0);
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
