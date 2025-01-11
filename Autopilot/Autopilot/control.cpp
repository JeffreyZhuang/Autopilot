#include "control.h"

Control::Control(HAL * hal, Plane * plane) : roll_controller(1, 0, 0, 0, 1),
											 pitch_controller(1, 0, 0, 0, 1),
											 yaw_controller(1, 0, 0, 0, 0),
											 alt_controller(1, 0, 0, 0, 0),
											 hdg_controller(1, 0, 0, 0, 0)
{
	_hal = hal;
	_plane = plane;
}

// Use guidance altitude and position setpoint to calculate control commands
void Control::update()
{
	// Direction to nearest setpoint
//	float heading_setpoint = atan(_plane->guidance_n_setpoint / _plane->guidance_e_setpoint);
//	float roll_setpoint = yaw_controller.get_output(_plane->ahrs_yaw, heading_setpoint, dt);
//	float pitch_setpoint = alt_controller.get_output(_plane->nav_pos_down, _plane->guidance_d_setpoint, dt);
	float roll_setpoint = 0;
	float pitch_setpoint = 0;

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
