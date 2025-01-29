#include "control.h"

#include <cstdio> // For testing, remove later

// Use PI controller for pitch and roll, then your integral term is the servo misalignment
Control::Control(HAL * hal, Plane * plane, float dt) : roll_controller(0.04, 0, 0, 0, -1, 1),
											 	 	   pitch_controller(0.04, 0, 0, 0, -1, 1),
													   hdg_controller(1, 0, 0, 0, -10, 10),
													   alt_controller(1, 0, 0, 0, -10, 10),
													   speed_controller(0.05, 0.01, 0, 0.3 / 0.01, -1, 1)
{
	_hal = hal;
	_plane = plane;
	_dt = dt;
}


void Control::update_manual()
{
	// Read from radio and direct to servos
	float rudder = _plane->rc_rudder;
	float elevator = _plane->rc_elevator;
	float throttle = _plane->rc_throttle;

	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(throttle);
}

// Hold roll and pitch at 0
void Control::update_stabilized()
{
	// Calculate control outputs
	float rudder = roll_controller.get_output(_plane->ahrs_roll,
											  0,
											  _dt / 1000000);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch,
												 0,
												 _dt / 1000000);

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
}

void Control::update_takeoff()
{
	float roll_setpoint = 0;
	float pitch_setpoint = 10;

	float rudder = roll_controller.get_output(_plane->ahrs_roll,
											  roll_setpoint,
											  _dt / 1000000);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch,
												 pitch_setpoint,
												 _dt / 1000000);
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
}

// Use guidance altitude and position setpoint to calculate control commands
void Control::update_mission()
{
	// Direction to nearest setpoint
	//	float heading_setpoint = atan(_plane->guidance_n_setpoint / _plane->guidance_e_setpoint);
	float heading_setpoint = 100;
	float roll_setpoint = hdg_controller.get_output(_plane->ahrs_yaw - 180.0f, heading_setpoint - 180.0f, _dt / 1000000);
	float pitch_setpoint = -alt_controller.get_output(_plane->nav_pos_down, _plane->guidance_d_setpoint, _dt / 1000000);

	// Calculate control outputs
	float rudder = roll_controller.get_output(_plane->ahrs_roll,
											  roll_setpoint,
											  _dt / 1000000);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch,
												 pitch_setpoint,
												 _dt / 1000000);
	float throttle = _plane->cruise_throttle + speed_controller.get_output(_plane->nav_airspeed,
												 _plane->airspeed_cruise,
												 _dt / 1000000);
	throttle = clamp(throttle, 0, 1);

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(throttle);
}

void Control::update_land()
{
	float roll_setpoint = 0;
	float pitch_setpoint = 10;

	float rudder = roll_controller.get_output(_plane->ahrs_roll,
											  roll_setpoint,
											  _dt / 1000000);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch,
												 pitch_setpoint,
												 _dt / 1000000);
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
}

float Control::clamp(float n, float min, float max)
{
    if (n > max)
    {
        n = max;
    }

    if (n < min)
    {
        n = min;
    }

    return n;
}
