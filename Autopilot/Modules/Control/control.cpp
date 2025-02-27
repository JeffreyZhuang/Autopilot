#include "control.h"

#include <cstdio>

// Use PI controller for pitch and roll, then your integral term is the servo misalignment
Control::Control(HAL * hal, Plane * plane, float dt)
	: roll_controller(false, hal->get_main_dt()),
	  pitch_controller(false, hal->get_main_dt()),
	  hdg_controller(true, hal->get_main_dt()),
	  alt_controller(false, hal->get_main_dt()),
	  speed_controller(false, hal->get_main_dt()), // Start with P first, then add I
	  _tecs(plane)
{
	_hal = hal;
	_plane = plane;
	_dt = dt;
}

// Read from radio and send commands directly to servos
void Control::update_manual()
{
	_plane->aileron_setpoint = _plane->rc_in_norm[params.aileron_ch];
	_plane->elevator_setpoint = _plane->rc_in_norm[params.elevator_ch];
	_plane->throttle_setpoint = _plane->rc_in_norm[params.throttle_ch];
}

// Pilot commands roll and pitch angles, throttle is manual
void Control::update_stabilized()
{
	_plane->pitch_setpoint = _plane->rc_in_norm[params.elevator_ch] * params.ptch_lim_deg;
	_plane->roll_setpoint = _plane->rc_in_norm[params.aileron_ch] * params.roll_lim_deg;
	_plane->aileron_setpoint = roll_controller.get_output(_plane->ahrs_roll, _plane->roll_setpoint, params.roll_kp, 0, 0, 0, -1, 1, 0);
	_plane->elevator_setpoint = pitch_controller.get_output(_plane->ahrs_pitch, _plane->pitch_setpoint, params.ptch_kp, 0.00002, 0, 1, -1, 1, 0);
	_plane->throttle_setpoint = _plane->rc_in_norm[params.throttle_ch];
}

// Set throttle to TAKEOFF_THR, hold a pitch angle of TAKEOFF_PTCH and a roll angle of 0
void Control::update_takeoff()
{
	_plane->pitch_setpoint = params.takeoff_ptch;
	_plane->roll_setpoint = 0;
	_plane->aileron_setpoint = roll_controller.get_output(_plane->ahrs_roll, _plane->roll_setpoint, params.roll_kp, 0, 0, 0, -1, 1, 0);
	_plane->elevator_setpoint = pitch_controller.get_output(_plane->ahrs_pitch, _plane->pitch_setpoint, params.ptch_kp, 0.00002, 0, 1, -1, 1, 0);
	_plane->throttle_setpoint = params.takeoff_thr;
}

// Track guidance altitude and heading setpoints at a speed of AIRSPEED_CUIRSE
void Control::update_mission()
{
	_tecs.update(params.aspd_cruise, _plane->guidance_d_setpoint, 1);
	_plane->pitch_setpoint = alt_controller.get_output(1, 0, 0, 0, -params.ptch_lim_deg, params.ptch_lim_deg, 0, _plane->tecs_error_diff, 0);
	_plane->roll_setpoint = hdg_controller.get_output(1, 0, 0, 0, -params.roll_lim_deg, params.roll_lim_deg, 0, _plane->ahrs_yaw, _plane->guidance_hdg_setpoint);
	_plane->aileron_setpoint = roll_controller.get_output(params.roll_kp, 0, 0, 0, -1, 1, 0, _plane->ahrs_roll, _plane->roll_setpoint);
	_plane->elevator_setpoint = pitch_controller.get_output(params.ptch_kp, 0.00002, 0, 1, -1, 1, 0, _plane->ahrs_pitch, _plane->pitch_setpoint);
	_plane->throttle_setpoint = speed_controller.get_output(params.thr_kp, 0.0001, 0, 1, 0, 1, params.throttle_cruise, _plane->tecs_error_total, 0);
}

// Track approach guidance altitude and heading setpoints at the reduced speed of AIRSPEED_LANDING
void Control::update_land()
{
	_tecs.update(params.aspd_land, _plane->guidance_d_setpoint, 1);
	_plane->pitch_setpoint = alt_controller.get_output(1, 0, 0, 0, -params.ptch_lim_deg, params.ptch_lim_deg, 0, _plane->tecs_error_diff, 0);
	_plane->roll_setpoint = hdg_controller.get_output(1, 0, 0, 0, -params.roll_lim_deg, params.roll_lim_deg, 0, _plane->ahrs_yaw, _plane->guidance_hdg_setpoint);
	_plane->aileron_setpoint = roll_controller.get_output(params.roll_kp, 0, 0, 0, -1, 1, 0, _plane->ahrs_roll, _plane->roll_setpoint);
	_plane->elevator_setpoint = pitch_controller.get_output(params.ptch_kp, 0.00002, 0, 1, -1, 1, 0, _plane->ahrs_pitch, _plane->pitch_setpoint);
	_plane->throttle_setpoint = speed_controller.get_output(params.thr_kp, 0.0001, 0, 1, 0, 1, params.throttle_cruise, _plane->tecs_error_total, 0);
}

// Cut throttle, set roll to 0 and track flare guidance altitude setpoints
void Control::update_flare()
{
	_tecs.update(params.aspd_land, _plane->guidance_d_setpoint, 2);
	_plane->roll_setpoint = 0;
	_plane->pitch_setpoint = alt_controller.get_output(1, 0, 0, 0, -params.ptch_lim_deg, params.ptch_lim_deg, 0, _plane->tecs_error_diff, 0);
	_plane->aileron_setpoint = roll_controller.get_output(params.roll_kp, 0, 0, 0, -1, 1, 0, _plane->ahrs_roll, _plane->roll_setpoint);
	_plane->elevator_setpoint = pitch_controller.get_output(params.ptch_kp, 0.00002, 0, 1, -1, 1, 0, _plane->ahrs_pitch, _plane->pitch_setpoint);
	_plane->throttle_setpoint = 0;
}
