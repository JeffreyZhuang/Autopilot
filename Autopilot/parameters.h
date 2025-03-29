#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdint.h>
#include <cstring>

struct __attribute__((packed)) Parameters
{
	// Performance
	struct __attribute__((packed))
	{
		float throttle_cruise; // Steady-state cruise throttle
	} perf;

	// Attitude control
	struct __attribute__((packed))
	{
		float ptch_kp;
		float ptch_ki;
		float roll_kp;
		float roll_ki;
	} att_ctrl;

	// Takeoff
	struct __attribute__((packed))
	{
		float alt; // Altitude to detect when takeoff is complete meters
		float ptch; // Pitch during takeoff
		float roll_lim; // Roll limit during takeoff
	} takeoff;

	// Landing
	struct __attribute__((packed))
	{
		float flare_alt; // Altitude to start flare
		float flare_sink_rate; // Target sink rate for flare, meters per second
		float touchdown_speed; // Detect touchdown when speed below this value in meters per second
	} landing;

	// TECS
	struct __attribute__((packed))
	{
		float aspd_cruise; // Meters per second
		float aspd_land; // Landing airspeed
		float min_aspd_mps; // Minimum airspeed meters per second
		float max_aspd_mps;
		float total_energy_kp;
		float total_energy_ki;
		float energy_balance_kp;
		float energy_balance_ki;
		float ptch_lim_deg; // Maximum pitch angle
	} tecs;

	// L1 Controller
	struct __attribute__((packed))
	{
		float period;
		float roll_lim; // Maximum roll angle in either direction
	} l1_ctrl;

	// Navigator
	struct __attribute__((packed))
	{
		float min_dist_wp; // Distance in meters from waypoint until switching to next
	} navigator;

	// Optical flow
	struct __attribute__((packed))
	{
		bool enable_of; // Enable optical flow
		int16_t of_min; // Maximum optical flow value in pixels per sec
		int16_t of_max;
	} sensors;

	// Mixer
	struct __attribute__((packed))
	{
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
	} mixer;

	// RC Transmitter Input
	struct __attribute__((packed))
	{
		uint16_t max_duty; // RC Transmitter stick max duty cycle microseconds
		uint16_t min_duty; // Make sure values are INSIDE the range of radio, NEVER outside
	} rc_input;

	// AHRS
	struct __attribute__((packed))
	{
		float beta_gain; // Madgwick filter gain
		float mag_decl; // Declination in degrees, determined from online calculator
		float acc_max; // Max acceleration in units of g to enable sensor fusion
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
	} ahrs;

	// Position Estimator
	struct __attribute__((packed))
	{
		float baro_var; // Variance
		float gnss_var;
	} pos_estimator;

	// HITL
	struct __attribute__((packed))
	{
		bool enable;
	} hitl;
};

const Parameters* get_params();
void set_params(const Parameters* params);
bool are_params_set();

#endif /* PARAMETERS_H_ */
