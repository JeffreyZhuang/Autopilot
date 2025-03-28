#include "autopilot.h"

Autopilot::Autopilot(HAL* hal, Data_bus* data_bus)
	: _ahrs(hal, data_bus),
	  _position_estimator(hal, data_bus),
	  _att_control(hal, data_bus),
	  _l1_controller(hal, data_bus),
	  _telem(hal, data_bus),
	  _storage(hal, data_bus),
	  _mixer(hal, data_bus),
	  _rc_handler(hal, data_bus),
	  _commander(hal, data_bus),
	  _tecs(hal, data_bus),
	  _navigator(hal, data_bus),
	  _time_pub(data_bus->time_node)
{
	_hal = hal;
	_data_bus = data_bus;
}

void Autopilot::setup()
{
	printf("Autopilot: Setup\n");
	_hal->init();
	_hal->start_main_task(&Autopilot::static_main_task, this);
	_hal->start_background_task(&Autopilot::static_background_task, this);
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

	if (_time_data.timestamp > 0)
	{
		_time_data.dt_s = (time - _time_data.timestamp) * US_TO_S;
	}
	else
	{
		// Initialize
		_time_data.dt_s = 0;
	}

	_time_data.timestamp = time;
	_time_data.loop_iteration++;

	_time_pub.publish(_time_data);
}

// View in web serial plotter
void Autopilot::debug_serial()
{
	// Maybe move to debug class
	if (_data_bus->modes_node.get(nullptr).system_mode != System_mode::CONFIG)
	{
		Pos_est_data pos_est_data = _data_bus->pos_est_node.get(nullptr);
		GNSS_data gnss_data = _data_bus->gnss_node.get(nullptr);
		AHRS_data ahrs_data = _data_bus->ahrs_node.get(nullptr);
		Baro_data baro_data = _data_bus->baro_node.get(nullptr);

		// Or serial class
		// Printf only for printing
		// USB class for USB
		// Or send this along with HITL
		double gnss_north_meters, gnss_east_meters;
		lat_lon_to_meters(_data_bus->telem_node.get(nullptr).waypoints[0].lat,
						  _data_bus->telem_node.get(nullptr).waypoints[0].lon,
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
				-(baro_data.alt - pos_est_data.baro_offset),
				ahrs_data.yaw);

		_hal->usb_print(tx_buff);
	}
}
