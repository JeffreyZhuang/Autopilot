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
	  _sensors(hal, data_bus)
{
	_hal = hal;

	init_params();
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
	_sensors.update();
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
}

void Autopilot::background_task()
{
	_storage.update_background();
}
