#include "autopilot.h"

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal, Data_bus* data_bus)
	: _ahrs(hal, data_bus),
	  _position_estimator(hal, data_bus),
	  _att_control(hal, data_bus),
	  _position_control(hal, data_bus),
	  _telem(hal, data_bus),
	  _storage(hal, data_bus),
	  _mixer(hal, data_bus),
	  _rc_handler(hal, data_bus),
	  _commander(hal, data_bus),
	  _navigator(hal, data_bus),
	  _sensors(hal, data_bus),
	  _usb_comm(hal, data_bus)
{
	_hal = hal;
	_instance = this;

	param_init();
}

// Create file when switching from disarmed to armed (switching from startup to flight)
// So it already has GPS fix or recieves time from GCS in that case
// File format: YYYY-MM-DD-HH-MM-SS

// How to get parameters. Do it in update_parameters function? Or

void Autopilot::setup()
{
	printf("Autopilot: Setup\n");

	_hal->init();
	_hal->set_main_task(&Autopilot::static_main_task);
}

void Autopilot::main_task()
{
	_sensors.update();
	_rc_handler.update();
	_ahrs.update();
	_position_estimator.update();
	_commander.update();
	_navigator.update();
	_position_control.update();
	_att_control.update();
	_mixer.update();
	_storage.update();
	_telem.update();
	_usb_comm.update();
}
