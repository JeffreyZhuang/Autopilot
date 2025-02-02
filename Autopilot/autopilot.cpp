#include <autopilot.h>

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
	_hal->write_storage_buffer();
	_telem.update();
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
	_ahrs.set_state(); // Set initial state

	// Calibrate barometer
	_plane->baro_offset = _plane->baro_alt;

	// Set home position to first GPS fix
	bool gnss_locked = (_plane->gnss_sats > 5) && (fabs(_plane->gnss_lat) > 0) && (fabs(_plane->gnss_lat) > 0);
	if (gnss_locked)
	{
		_plane->gnss_center_lat = _plane->gnss_lat;
		_plane->gnss_center_lon = _plane->gnss_lon;
	}

	bool transmitter_safe = (_plane->rc_throttle < 0.1) && (_plane->manual_sw == true);

	if (gnss_locked && transmitter_safe)
	{
		_plane->systemMode = SystemMode::FLIGHT;
	}
}

void Autopilot::flight()
{
	_ahrs.update();
	_navigation.execute();

	_guidance.update();

	if (_plane->manual_sw)
	{
		evaluate_auto_mode();
	}
	else
	{
		evaluate_manual_mode();
	}
}

/**
 * Auto Modes
 */
void Autopilot::evaluate_auto_mode()
{
	switch (_plane->autoMode)
	{
	case AutoMode::TAKEOFF:
		takeoff();
		break;
	case AutoMode::MISSION:
		mission();
		break;
	case AutoMode::LAND:
		land();
		break;
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
	_control.update_mission();
}

void Autopilot::land()
{
	_control.update_land();
}

/**
 * Manual Modes
 */
void Autopilot::evaluate_manual_mode()
{
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
	_plane->autoMode = AutoMode::TAKEOFF;
}

void Autopilot::update_time()
{
	uint64_t time = _hal->get_time_us();
	_plane->loop_execution_time = time - _plane->time;
	_plane->time = time;
	_plane->loop_iteration++;
}
