#include "control.h"

#include <cstdio>

// Use PI controller for pitch and roll, then your integral term is the servo misalignment
Control::Control(HAL * hal, Plane * plane, float dt)
	: roll_controller(ROLL_KP, 0, 0, 0, -1, 1, 0, false),
	  pitch_controller(PTCH_KP, 0.00002, 0, 1, -1, 1, 0, false),
	  hdg_controller(1, 0, 0, 0, -ROLL_LIM_DEG, ROLL_LIM_DEG, 0, true),
	  alt_controller(1, 0, 0, 0, PTCH_LIM_MIN_DEG, PTCH_LIM_MAX_DEG, 0, false),
	  speed_controller(THR_KP, 0.0001, 0, 1, 0, 1, TRIM_THROTTLE, false), // Start with P first, then add I
	  _tecs(plane)
{
	_hal = hal;
	_plane = plane;
	_dt = dt;
}

// Read from radio and direct to servos
void Control::update_manual()
{
	_plane->aileron_setpoint = _plane->rc_rudder;
	_plane->elevator_setpoint = _plane->rc_elevator;

	if (_plane->rc_throttle < THR_DEADZONE)
	{
		_plane->throttle_setpoint = 0;
	}
	else
	{
		_plane->throttle_setpoint = _plane->rc_throttle;
	}
}

// Pilot commands roll and pitch angles. Throttle is manual
void Control::update_stabilized()
{
	_plane->roll_setpoint = _plane->rc_rudder * ROLL_LIM_DEG;
	_plane->pitch_setpoint = _plane->rc_elevator * PTCH_LIM_MAX_DEG;
	track_roll_pitch_sp();
	_plane->throttle_setpoint = _plane->rc_throttle;
}

void Control::update_takeoff()
{
	_plane->pitch_setpoint = TAKEOFF_PTCH;
	_plane->roll_setpoint = 0;
	track_roll_pitch_sp();
	_plane->throttle_setpoint = TAKEOFF_THR;
}

void Control::update_mission()
{
	_tecs.update(AIRSPEED_CRUISE, _plane->guidance_d_setpoint, 1);
	track_tecs_alt_thr();
	_plane->roll_setpoint = hdg_controller.get_output(_plane->ahrs_yaw, _plane->guidance_hdg_setpoint, _dt);
	track_roll_pitch_sp();
}

void Control::update_land()
{
	_tecs.update(AIRSPEED_LANDING, _plane->guidance_d_setpoint, 1);
	track_tecs_alt_thr();
	_plane->roll_setpoint = hdg_controller.get_output(_plane->ahrs_yaw, _plane->guidance_hdg_setpoint, _dt);
	track_roll_pitch_sp();
}

void Control::update_flare()
{
	_tecs.update(AIRSPEED_LANDING, _plane->guidance_d_setpoint, 2);
	_plane->roll_setpoint = 0;
	_plane->pitch_setpoint = alt_controller.get_output(_plane->tecs_error_diff, 0, _dt);
	track_roll_pitch_sp();
	_plane->throttle_setpoint = 0;
}

/**
 * Helper functions
 */
void Control::track_roll_pitch_sp()
{
	_plane->aileron_setpoint = roll_controller.get_output(_plane->ahrs_roll, _plane->roll_setpoint, _dt);
	_plane->elevator_setpoint = pitch_controller.get_output(_plane->ahrs_pitch, _plane->pitch_setpoint, _dt);
}

void Control::track_tecs_alt_thr()
{
	_plane->pitch_setpoint = alt_controller.get_output(_plane->tecs_error_diff, 0, _dt);
	_plane->throttle_setpoint = speed_controller.get_output(_plane->tecs_error_total, 0, _dt);
}
