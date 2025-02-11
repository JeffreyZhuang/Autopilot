#include "control.h"

#include <cstdio>

// Use PI controller for pitch and roll, then your integral term is the servo misalignment
Control::Control(HAL * hal, Plane * plane, float dt) : roll_controller(0.04, 0, 0, 0, -1, 1, 0, false),
											 	 	   pitch_controller(0.04, 0.00002, 0, 1, -1, 1, 0, false),
													   hdg_controller(1, 0, 0, 0, -ROLL_LIM_DEG, ROLL_LIM_DEG, 0, true),
													   alt_controller(1, 0, 0, 0, PTCH_LIM_MIN_DEG, PTCH_LIM_MAX_DEG, 0, false),
													   speed_controller(0.01, 0.0001, 0, 1, 0, 1, TRIM_THROTTLE, false) // Start with P first, then add I
{
	_hal = hal;
	_plane = plane;
	_dt = dt;
}

// Read from radio and direct to servos
void Control::update_manual()
{
	float rudder = _plane->rc_rudder;
	float elevator = _plane->rc_elevator;
	float throttle = _plane->rc_throttle;

	if (throttle < THR_DEADZONE)
	{
		throttle = 0;
	}

	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(throttle);
}

// Pilot commands roll and pitch angles. Throttle is manual
void Control::update_stabilized()
{
	float roll_setpoint = _plane->rc_rudder * ROLL_LIM_DEG;
	float pitch_setpoint = _plane->rc_elevator * PTCH_LIM_MAX_DEG;

	// Calculate control outputs
	float rudder = roll_controller.get_output(_plane->ahrs_roll, roll_setpoint, _dt);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch, pitch_setpoint, _dt);
	float throttle = _plane->rc_throttle;

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(throttle);
}

void Control::update_takeoff()
{
	float rudder = roll_controller.get_output(_plane->ahrs_roll, 0, _dt);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch, TAKEOFF_PTCH, _dt);

	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(TAKEOFF_THR);
}

// Use guidance altitude and position setpoint to calculate control commands
void Control::update_mission()
{
	// Calculate roll and pitch setpoints to reach waypoint
	float roll_setpoint = hdg_controller.get_output(_plane->ahrs_yaw, _plane->guidance_hdg_setpoint, _dt);
	float pitch_setpoint = -alt_controller.get_output(_plane->nav_pos_down, _plane->guidance_d_setpoint, _dt);

	// Calculate control outputs to track roll and pitch setpoints
	float rudder = roll_controller.get_output(_plane->ahrs_roll, roll_setpoint, _dt);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch, pitch_setpoint, _dt);
	float throttle = speed_controller.get_output(_plane->nav_airspeed, AIRSPEED_CRUISE, _dt);

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(throttle);
}

void Control::update_land()
{
	// Calculate roll and pitch setpoints to reach waypoint
	float roll_setpoint = hdg_controller.get_output(_plane->ahrs_yaw, _plane->guidance_hdg_setpoint, _dt);
	float pitch_setpoint = -alt_controller.get_output(_plane->nav_pos_down, _plane->guidance_d_setpoint, _dt);

	// Calculate control outputs to track roll and pitch setpoints
	float rudder = roll_controller.get_output(_plane->ahrs_roll, roll_setpoint, _dt);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch, pitch_setpoint, _dt);
	float throttle = speed_controller.get_output(_plane->nav_airspeed, AIRSPEED_LANDING, _dt);

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(throttle);
}

// Pitch dips because throttle cut
void Control::update_flare()
{
	// Calculate roll and pitch setpoints to reach waypoint
	float roll_setpoint = 0;
	float pitch_setpoint = -alt_controller.get_output(_plane->nav_pos_down, _plane->guidance_d_setpoint, _dt);

	// Calculate control outputs to track roll and pitch setpoints
	float rudder = roll_controller.get_output(_plane->ahrs_roll, roll_setpoint, _dt);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch, pitch_setpoint, _dt);

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(0);
}
