#include "control.h"

Control::Control(HAL * hal, Plane * plane, float dt) : roll_controller(0.03, 0, 0, 0, 1),
											 	 	   pitch_controller(0.03, 0, 0, 0, 1),
													   hdg_controller(0.01, 0, 0, 0, 20),
													   alt_controller(0.01, 0, 0, 0, 20)
{
	_hal = hal;
	_plane = plane;
	_dt = dt;
}


void Control::update()
{
	cruise();

	switch (_plane->flightState)
	{
	case FlightState::MANUAL:
		break;
	case FlightState::CRUISE:
		break;
	case FlightState::TAKEOFF:
		break;
	case FlightState::LAND:
		break;
	case FlightState::STABALIZE:
		break;
	}
}

void Control::manual()
{
	// Read from radio and direct to servos
	float rudder = 0;
	float elevator = 0;

	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
}

void Control::takeoff()
{

}

// Use guidance altitude and position setpoint to calculate control commands
void Control::cruise()
{
	// Direction to nearest setpoint
	//	float heading_setpoint = atan(_plane->guidance_n_setpoint / _plane->guidance_e_setpoint);
	float heading_setpoint = 0;
	float roll_setpoint = hdg_controller.get_output(_plane->ahrs_yaw, heading_setpoint, _dt);
	float pitch_setpoint = alt_controller.get_output(_plane->nav_pos_down, _plane->guidance_d_setpoint, _dt);

	// Calculate control outputs
	float rudder = roll_controller.get_output(_plane->ahrs_roll,
											  roll_setpoint,
											  _dt);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch,
												 pitch_setpoint,
												 _dt);

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
}

void Control::land()
{

}

// Hold roll and pitch at 0
void Control::stabalize()
{
	// Calculate control outputs
	float rudder = roll_controller.get_output(_plane->ahrs_roll,
											  0,
											  _dt);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch,
												 0,
												 _dt);

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
}
