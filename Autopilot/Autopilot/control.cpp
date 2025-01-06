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
	float heading_setpoint = 0; // Direction to nearest setpoint

	float roll_setpoint = yaw_controller.get_output(_plane->ahrs_yaw, heading_setpoint, dt);
	float pitch_setpoint = alt_controller.get_output(_plane->nav_pos_d, _plane->guidance_d_setpoint, dt);

	// Calculate control outputs
	float rudder = roll_controller.get_output(_plane->ahrs_roll,
											  roll_setpoint,
											  dt);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch,
												 pitch_setpoint,
												 dt);

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
}
