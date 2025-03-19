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

	// Sensors
	bool enable_of; // Enable optical flow

	// Guidance
	float guidance_kp; // Proportional guidance law gain

	// Landing
	float flare_alt; // Altitude to start flare
	float flare_sink_rate; // Target sink rate for flare, meters per second
	float touchdown_aspd_thresh; // Detect touchdown when speed below this value in meters per second

	// Takeoff
	float takeoff_alt; // Altitude to detect when takeoff is complete meters
	float takeoff_ptch; // Pitch during takeoff
	float takeoff_roll_lim; // Degrees

	// Stabilization
	float stab_ptch_lim;
	float stab_roll_lim;

	// Mixer
	uint16_t pwm_max_ele;
	uint16_t pwm_max_rud;
	uint16_t pwm_max_thr;
	uint16_t pwm_max_aux1;
	uint16_t pwm_max_aux2;
	uint16_t pwm_max_aux3;
	uint16_t pwm_min_ele;
	uint16_t pwm_min_rud;
	uint16_t pwm_min_thr;
	uint16_t pwm_min_aux1;
	uint16_t pwm_min_aux2;
	uint16_t pwm_min_aux3;
	bool pwm_rev_ele;
	bool pwm_rev_rud;
	bool pwm_rev_thr;
	bool pwm_rev_aux1;
	bool pwm_rev_aux2;
	bool pwm_rev_aux3;

	// RC Transmitter
	uint16_t rc_max; // RC Transmitter stick max duty cycle microseconds
	uint16_t rc_min; // Make sure values are INSIDE the range of radio, NEVER outside

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
	float hard_iron_x;
	float hard_iron_y;
	float hard_iron_z;
	float soft_iron_xx;
	float soft_iron_xy;
	float soft_iron_xz;
	float soft_iron_yx;
	float soft_iron_yy;
	float soft_iron_yz;
	float soft_iron_zx;
	float soft_iron_zy;
	float soft_iron_zz;

	// Kalman filter
	float baro_r; // Variance
	float gnss_r;
	float gnss_alt_r;

	// HITL
	bool enable_hitl;
};

const Parameters* get_params();
void set_params(const Parameters* params);
bool are_params_set();

#endif /* PARAMETERS_H_ */
