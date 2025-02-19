#include "control.h"

#include <cstdio>

// Use PI controller for pitch and roll, then your integral term is the servo misalignment
Control::Control(HAL * hal, Plane * plane, float dt)
	: roll_controller(ROLL_KP, 0, 0, 0, -1, 1, 0, false),
	  pitch_controller(PTCH_KP, 0.00002, 0, 1, -1, 1, 0, false),
	  hdg_controller(1, 0, 0, 0, -ROLL_LIM_DEG, ROLL_LIM_DEG, 0, true),
	  alt_controller(1, 0, 0, 0, -PTCH_LIM_DEG, PTCH_LIM_DEG, 0, false),
	  speed_controller(THR_KP, 0.0001, 0, 1, 0, 1, THROTTLE_CRUISE, false), // Start with P first, then add I
	  _tecs(plane)
{
	_hal = hal;
	_plane = plane;
	_dt = dt;
}

// Read from radio and send commands directly to servos
void Control::update_manual()
{
	_plane->aileron_setpoint = _plane->rc_rudder;
	_plane->elevator_setpoint = _plane->rc_elevator;
	_plane->throttle_setpoint = _plane->rc_throttle;
}

// Pilot commands roll and pitch angles, throttle is manual
void Control::update_stabilized()
{
	_plane->pitch_setpoint = _plane->rc_elevator * PTCH_LIM_DEG;
	_plane->roll_setpoint = _plane->rc_rudder * ROLL_LIM_DEG;
	_plane->aileron_setpoint = roll_controller.get_output(_plane->ahrs_roll, _plane->roll_setpoint, _dt);
	_plane->elevator_setpoint = pitch_controller.get_output(_plane->ahrs_pitch, _plane->pitch_setpoint, _dt);
	_plane->throttle_setpoint = _plane->rc_throttle;
}

// Set throttle to TAKEOFF_THR, hold a pitch angle of TAKEOFF_PTCH and a roll angle of 0
void Control::update_takeoff()
{
	_plane->pitch_setpoint = TAKEOFF_PTCH;
	_plane->roll_setpoint = 0;
	_plane->aileron_setpoint = roll_controller.get_output(_plane->ahrs_roll, _plane->roll_setpoint, _dt);
	_plane->elevator_setpoint = pitch_controller.get_output(_plane->ahrs_pitch, _plane->pitch_setpoint, _dt);
	_plane->throttle_setpoint = TAKEOFF_THR;
}

// Track guidance altitude and heading setpoints at a speed of AIRSPEED_CUIRSE
void Control::update_mission()
{
	_tecs.update(AIRSPEED_CRUISE, _plane->guidance_d_setpoint, 1);
	_plane->pitch_setpoint = alt_controller.get_output(_plane->tecs_error_diff, 0, _dt);
	_plane->roll_setpoint = hdg_controller.get_output(_plane->ahrs_yaw, _plane->guidance_hdg_setpoint, _dt);
	_plane->aileron_setpoint = roll_controller.get_output(_plane->ahrs_roll, _plane->roll_setpoint, _dt);
	_plane->elevator_setpoint = pitch_controller.get_output(_plane->ahrs_pitch, _plane->pitch_setpoint, _dt);
	_plane->throttle_setpoint = speed_controller.get_output(_plane->tecs_error_total, 0, _dt);
}

// Track approach guidance altitude and heading setpoints at the reduced speed of AIRSPEED_LANDING
void Control::update_land()
{
	_tecs.update(AIRSPEED_LANDING, _plane->guidance_d_setpoint, 1);
	_plane->pitch_setpoint = alt_controller.get_output(_plane->tecs_error_diff, 0, _dt);
	_plane->roll_setpoint = hdg_controller.get_output(_plane->ahrs_yaw, _plane->guidance_hdg_setpoint, _dt);
	_plane->aileron_setpoint = roll_controller.get_output(_plane->ahrs_roll, _plane->roll_setpoint, _dt);
	_plane->elevator_setpoint = pitch_controller.get_output(_plane->ahrs_pitch, _plane->pitch_setpoint, _dt);
	_plane->throttle_setpoint = speed_controller.get_output(_plane->tecs_error_total, 0, _dt);
}

// Cut throttle, set roll to 0 and track flare guidance altitude setpoints
void Control::update_flare()
{
	_tecs.update(AIRSPEED_LANDING, _plane->guidance_d_setpoint, 2);
	_plane->roll_setpoint = 0;
	_plane->pitch_setpoint = alt_controller.get_output(_plane->tecs_error_diff, 0, _dt);
	_plane->aileron_setpoint = roll_controller.get_output(_plane->ahrs_roll, _plane->roll_setpoint, _dt);
	_plane->elevator_setpoint = pitch_controller.get_output(_plane->ahrs_pitch, _plane->pitch_setpoint, _dt);
	_plane->throttle_setpoint = 0;
}
