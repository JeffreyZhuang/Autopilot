#include "autopilot.h"

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal, Plane* plane)
	: _ahrs(hal, plane, hal->get_main_dt()),
	  _navigation(hal, plane, hal->get_main_dt()),
	  _control(hal, plane, hal->get_main_dt()),
	  _guidance(hal, plane),
	  _telem(hal, plane),
	  _storage(plane, hal),
	  _control_allocator(hal, plane),
	  _rc_handler(hal, plane)
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

	printf("Start\n");

	// Start tasks
	_hal->set_main_task(&Autopilot::static_main_task);
	_hal->set_background_task(&Autopilot::static_logger_task);
}

// High priority
void Autopilot::main_task()
{
	if (params.set)
	{
		update_time();
		_hal->read_sensors();
		_rc_handler.rc_update();

		evaluate_system_mode();

		_storage.write();
	}

	_telem.update();
}

// Runs at a lower priority
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

		if (_navigation.set_home())
		{
			_navigation.execute();
		}
	}

	bool waypoints_loaded = _plane->num_waypoints > 0;
	bool transmitter_safe = _plane->rc_in_norm[params.throttle_ch] == 0 && !_plane->manual_sw && !_plane->mode_sw;
	if (_plane->gps_fix && transmitter_safe && waypoints_loaded && _plane->tx_connected)
	{
		_plane->systemMode = SystemMode::FLIGHT;
	}

	debug_serial();
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

	_control_allocator.update();
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
	if ((_plane->rc_in_norm[params.throttle_ch] == 1) || (-_plane->nav_pos_down > params.takeoff_alt))
	{
		_plane->autoMode = AutoMode::TAKEOFF;
	}
}

void Autopilot::takeoff()
{
	_control.update_takeoff();

	if (-_plane->nav_pos_down > params.takeoff_alt)
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

	if (_plane->rangefinder_dist < params.land_flare_alt)
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

	// Detect touchdown when speed is below TOUCHDOWN_SPD_THR
	if (_plane->nav_airspeed < params.touchdown_aspd_thresh)
	{
		_plane->autoMode = AutoMode::TOUCHDOWN;
	}
}

void Autopilot::touchdown()
{
	_plane->aileron_setpoint = 0;
	_plane->elevator_setpoint = 0;
	_plane->throttle_setpoint = 0;
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

// View in web serial plotter
void Autopilot::debug_serial()
{
	double gnss_north_meters, gnss_east_meters; // Put this in gnss_hal.cpp?
	lat_lon_to_meters(_plane->home_lat,
					  _plane->home_lon,
					  _plane->gnss_lat,
					  _plane->gnss_lon,
					  &gnss_north_meters,
					  &gnss_east_meters);

	char tx_buff[200];
	sprintf(tx_buff,
			"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f\n",
			_plane->nav_acc_north,
			_plane->nav_acc_east,
			_plane->nav_acc_down,
			_plane->nav_vel_north,
			_plane->nav_vel_east,
			_plane->nav_vel_down,
			_plane->nav_pos_north,
			_plane->nav_pos_east,
			_plane->nav_pos_down,
			gnss_north_meters,
			gnss_east_meters,
			-(_plane->baro_alt - _plane->baro_offset),
			_plane->ahrs_yaw);
	_hal->usb_print(tx_buff);
}
