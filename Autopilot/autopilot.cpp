#include "autopilot.h"

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal, Plane* plane)
	: _ahrs(hal, plane),
	  _position_estimator(hal, plane),
	  _att_control(hal, plane),
	  _l1_controller(hal, plane),
	  _telem(hal, plane),
	  _storage(hal, plane),
	  _mixer(hal, plane),
	  _rc_handler(hal, plane),
	  _commander(hal, plane),
	  _tecs(hal, plane),
	  _navigator(hal, plane)
{
	_hal = hal;
	_plane = plane;
	_instance = this;
}

void Autopilot::setup()
{
	printf("Autopilot: Setup\n");
	_hal->init();
	_hal->start_main_task(&Autopilot::static_main_task);
	_hal->start_background_task(&Autopilot::static_background_task);
}

void Autopilot::main_task()
{
	update_time();
	_hal->read_sensors();
	_rc_handler.update();
	_ahrs.update();
	_position_estimator.update();
	_commander.update();
	_navigator.update();
	_l1_controller.update();
	_tecs.update();
	_att_control.update();
	_mixer.update();
	_storage.update();
	_telem.update();
	debug_serial();
}

void Autopilot::background_task()
{
	_storage.update_background();
}

/**
 * Helper functions
 */
void Autopilot::update_time()
{
	uint64_t time = _hal->get_time_us();

	if (_plane->time_us > 0)
	{
		_plane->dt_s = (time - _plane->time_us) * US_TO_S;
	}
	else
	{
		// Initialize
		_plane->dt_s = 0;
	}

	_plane->time_us = time;
	_plane->loop_iteration++;
}

// View in web serial plotter
void Autopilot::debug_serial()
{
	// Maybe move to debug class
	if (Data_bus::get_instance().modes_data.get(nullptr).system_mode != System_mode::CONFIG)
	{
		Pos_est_data pos_est_data = Data_bus::get_instance().pos_est_data.get(nullptr);

		// Or serial class
		// Printf only for printing
		// USB class for USB
		// Or send this along with HITL
		double gnss_north_meters, gnss_east_meters;
		lat_lon_to_meters(_plane->get_home_lat(), _plane->get_home_lon(),
						  gnss_data.lat, gnss_data.lon,
						  &gnss_north_meters, &gnss_east_meters);

		char tx_buff[200];
		sprintf(tx_buff,
				"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f\n",
				pos_est_data.vel_n,
				pos_est_data.vel_e,
				pos_est_data.vel_d,
				pos_est_data.pos_n,
				pos_est_data.pos_e,
				pos_est_data.pos_d,
				gnss_north_meters,
				gnss_east_meters,
				-(baro_data.alt - _plane->baro_offset),
				ahrs_data.yaw);

		_hal->usb_print(tx_buff);
	}
}
