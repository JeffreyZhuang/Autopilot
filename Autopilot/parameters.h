#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdint.h>

// Transmitter is detected if value higher than this
constexpr uint16_t TX_DETECT_MIN_DUTY = 500;

// Store in micro-SD but can be updated from GCS
struct __attribute__((packed))Parameters
{
	// If parameters have been set
	bool set = false;

	// Flight characteristics
	float aspd_cruise; // Meters per second
	float aspd_land; // Landing airspeed
	float tecs_min_aspd_mps; // Minimum airspeed meters per second
	float tecs_max_aspd_mps;
	float throttle_cruise; // Steady-state cruise throttle
	float ptch_lim_deg; // Maximum pitch in either direction
	float roll_lim_deg; // Maximum roll angle in either direction
	float min_dist_wp; // Distance in meters from waypoint until switching to next

	// Landing
	float land_gs_deg; // Approach glideslope angle
	float land_flare_alt; // Flare altitude
	float flare_sink_rate; // Meters per second
	float flare_trans_sec; // Time to transition to flare
	float touchdown_aspd_thresh; // Detect touchdown when speed below this value in meters per second

	// Takeoff
	float takeoff_alt; // Altitude to detect when takeoff is complete meters
	float takeoff_thr; // Throttle set during takeoff between 0 and 1
	float takeoff_ptch; // Pitch during takeoff

	// Mixer
	uint16_t max_duty[6]; // Max duty cycle in microseconds for each channel
	uint16_t min_duty[6];
	uint16_t rc_in_max; // RC Transmitter stick max duty cycle microseconds
	uint16_t rc_in_min; // Make sure values are INSIDE the range of radio, NEVER outside
	bool rev_ch[6]; // Reverse channels
	uint8_t aileron_ch; // Channel for aileron
	uint8_t elevator_ch; // Channel for elevator
	uint8_t throttle_ch; // Channel for throttle

	// RC Transmitter
	uint8_t manual_sw_ch;
	uint8_t mode_sw_ch;

	// PID
	float ptch_kp; // Proportional gain
	float roll_kp;
	float thr_kp;

	// AHRS
	float ahrs_beta; // Madwick filter gain
	float mag_decl; // Declination in degrees, determined from online calculator
	float ahrs_acc_max; // Max acceleration in units of g to enable sensor fusion
	float hard_iron[3];
	float soft_iron[3][3];

	// Kalman filter
	float baro_r; // Variance
	float gnss_r;
	float gnss_alt_r;
};

extern Parameters params;

#endif /* PARAMETERS_H_ */
