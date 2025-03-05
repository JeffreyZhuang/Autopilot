#include "autopilot.h"

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal, Plane* plane)
	: _ahrs(hal, plane),
	  _navigation(hal, plane),
	  _control(hal, plane),
	  _guidance(hal, plane),
	  _telem(hal, plane),
	  _storage(plane, hal),
	  _mixer(hal, plane),
	  _rc_handler(hal, plane),
	  _commander(hal, plane)
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
	update_time();
	_hal->read_sensors();
	_rc_handler.rc_update();
	_ahrs.update();
	_navigation.update();
	_commander.update();
	_guidance.update();
	_control.update();
	_mixer.update();
	_storage.write();
	_telem.update();

	debug_serial();
}

// Runs at a lower priority
void Autopilot::logger_task()
{
	_storage.flush();
}

/**
 * Helper functions
 */
void Autopilot::init_state()
{
	_plane->system_mode = System_mode::CONFIG;
	_plane->flight_mode = Flight_mode::MANUAL;
	_plane->manual_mode = Manual_mode::DIRECT;
	_plane->auto_mode = Auto_mode::TAKEOFF;
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
	double gnss_north_meters, gnss_east_meters;
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
