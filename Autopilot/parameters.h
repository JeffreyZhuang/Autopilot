#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdint.h>
#include <cstring>

struct __attribute__((packed))Parameters
{
	// Flight characteristics
	float aspd_cruise; // Meters per second
	float aspd_land; // Landing airspeed
	float tecs_min_aspd_mps; // Minimum airspeed meters per second
	float tecs_max_aspd_mps;
	float throttle_cruise; // Steady-state cruise throttle
	float ptch_lim_deg; // Maximum pitch in either direction
	float roll_lim_deg; // Maximum roll angle in either direction
	float min_dist_wp; // Distance in meters from waypoint until switching to next

	// Guidance
	float guidance_kp; // Proportional guidance law gain

	// Landing
	float flare_alt; // Altitude to start flare
	float flare_sink_rate; // Target sink rate for flare, meters per second
	float touchdown_aspd_thresh; // Detect touchdown when speed below this value in meters per second

	// Takeoff
	float takeoff_alt; // Altitude to detect when takeoff is complete meters
	float takeoff_ptch; // Pitch during takeoff

	// Mixer
	uint16_t max_duty[6]; // Max duty cycle in microseconds for each channel
	uint16_t min_duty[6];
	uint16_t rc_in_max; // RC Transmitter stick max duty cycle microseconds
	uint16_t rc_in_min; // Make sure values are INSIDE the range of radio, NEVER outside
	bool rev_ch[6]; // Reverse output PWM channels
	uint8_t aileron_ch; // Channel for aileron
	uint8_t elevator_ch; // Channel for elevator
	uint8_t throttle_ch; // Channel for throttle

	// RC Transmitter
	uint8_t manual_sw_ch;
	uint8_t mode_sw_ch;

	// PID gains
	float ptch_kp;
	float ptch_ki;
	float roll_kp;
	float roll_ki;
	float thr_kp;
	float thr_ki;
	float alt_kp;
	float alt_ki;
	float hdg_kp;
	float hdg_ki;

	// AHRS
	float ahrs_beta; // Madgwick filter gain
	float mag_decl; // Declination in degrees, determined from online calculator
	float ahrs_acc_max; // Max acceleration in units of g to enable sensor fusion
	float hard_iron[3];
	float soft_iron[3][3];

	// Kalman filter
	float baro_r; // Variance
	float gnss_r;
	float gnss_alt_r;
};

const Parameters* get_params();
void set_params(const Parameters* params);
bool are_params_set();

#endif /* PARAMETERS_H_ */
