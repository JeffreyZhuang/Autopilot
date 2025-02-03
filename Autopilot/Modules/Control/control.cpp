#include "control.h"

// Use PI controller for pitch and roll, then your integral term is the servo misalignment
Control::Control(HAL * hal, Plane * plane, float dt) : roll_controller(0.04, 0.005, 0, 0.5, -1, 1, false),
											 	 	   pitch_controller(0.04, 0, 0, 0, -1, 1, false),
													   hdg_controller(1, 0, 0, 0, -ROLL_LIM_DEG, ROLL_LIM_DEG, true),
													   alt_controller(1, 0, 0, 0, PTCH_LIM_MIN_DEG, PTCH_LIM_MAX_DEG, false),
													   speed_controller(0.05, 0.01, 0, 0.3, -1, 1, false)
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
	float rudder = roll_controller.get_output(_plane->ahrs_roll, roll_setpoint, _dt / 1000000);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch, pitch_setpoint, _dt / 1000000);
	float throttle = _plane->rc_throttle;

	// Set control surfaces
	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(throttle);
}

void Control::update_takeoff()
{
	float rudder = roll_controller.get_output(_plane->ahrs_roll,0, _dt / 1000000);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch, TAKEOFF_PTCH, _dt / 1000000);

	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(TAKEOFF_THR);
}

// Use guidance altitude and position setpoint to calculate control commands
void Control::update_mission()
{
	// Calculate roll and pitch setpoints to reach waypoint
	float roll_setpoint = hdg_controller.get_output(_plane->ahrs_yaw, _plane->guidance_hdg_setpoint, _dt / 1000000); // Convert range from (0, 360) to (-180, 180)
	float pitch_setpoint = -alt_controller.get_output(_plane->nav_pos_down, _plane->guidance_d_setpoint, _dt / 1000000);

	// Calculate control outputs to track roll and pitch setpoints
	float rudder = roll_controller.get_output(_plane->ahrs_roll, roll_setpoint, _dt / 1000000);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch, pitch_setpoint, _dt / 1000000);
	float throttle = TRIM_THROTTLE + speed_controller.get_output(_plane->nav_airspeed, AIRSPEED_CRUISE, _dt / 1000000);
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

	float rudder = roll_controller.get_output(_plane->ahrs_roll, roll_setpoint, _dt / 1000000);
	float elevator = pitch_controller.get_output(_plane->ahrs_pitch, pitch_setpoint, _dt / 1000000);

	_hal->set_elevator(elevator);
	_hal->set_rudder(rudder);
	_hal->set_throttle(0);
}
