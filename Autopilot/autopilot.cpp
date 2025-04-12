#include "autopilot.h"

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
	create_params();
}

void Autopilot::setup()
{
	printf("Autopilot: Setup\n");

	_hal->init();
	_hal->set_main_task(&Autopilot::static_main_task, this);
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
