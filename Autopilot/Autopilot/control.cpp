#include "control.h"

Control::Control(HAL * hal, Plane * plane) : roll_controller(0, 0, 0, 0, 0),
											 pitch_controller(0, 0, 0, 0, 0),
											 yaw_controller(0, 0, 0, 0, 0),
											 alt_controller(0, 0, 0, 0, 0),
											 hdg_controller(0, 0, 0, 0, 0)
{
	_hal = hal;
	_plane = plane;
}

void Control::update()
{
	roll_controller.get_output(_plane->ahrs_roll, _plane->roll_setpoint, dt);
	pitch_controller.get_output();
	yaw_controller.get_output();

	_hal->set_elevator(0);
	_hal->set_rudder(0);
}
