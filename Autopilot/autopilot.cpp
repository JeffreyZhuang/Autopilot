#include "autopilot.h"

// TODO:
// Aplink set param maybe use hash (uint32_t hash instead of uint8_t because collisions) instead of string for param name
// Use int32_t for value so you just set integer and union automatically converts to float, then you dont need array in code generator
// How to get parameters. Do it in update_parameters function? Or
// REmove reverse throttle
// Calibration
// Rename data bus
// Renaming classes in lib too
// Enabing calibration mode
// Use xyz instead of NED, z_setpoint instead of d_setpoint
// For personal website, put each image in container with desired width and height, then set image to 100% width and 100% height and object-fit: contain;
// Update simulator so it supports APLink HITL
// Its good for param set to keep checking if correct type, because if incorrect type then param_all_set will not be true
// Optical flow
// Datalogging

// By convention, the transmitter must be setup such that stick forward decreases PWM, and stick back increases PWM
// The PWM value direction corresponds to pitch direction
// Positive ele_norm variable means moving the elevator in the direction that causes a pitch up
// Positive elevator PWM also means moving the elevator in the direction that causes a pitch up

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal)
	: _ahrs(hal, &_data_bus),
	  _position_estimator(hal, &_data_bus),
	  _att_control(hal, &_data_bus),
	  _position_control(hal, &_data_bus),
	  _telem(hal, &_data_bus),
	  _storage(hal, &_data_bus),
	  _mixer(hal, &_data_bus),
	  _rc_handler(hal, &_data_bus),
	  _commander(hal, &_data_bus),
	  _navigator(hal, &_data_bus),
	  _sensors(hal, &_data_bus),
	  _usb_comm(hal, &_data_bus)
{
	_hal = hal;
	_instance = this;

	param_init();
}

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
