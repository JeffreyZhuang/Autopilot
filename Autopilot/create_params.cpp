#include "lib/parameters/parameters.h"

void init_params()
{
	// Attitude control
	param_add("ATT_PTCH_KP", PARAM_TYPE_FLOAT); // Pitch control proportional gain
	param_add("ATT_PTCH_KI", PARAM_TYPE_FLOAT); // Pitch control integral gain
	param_add("ATT_ROLL_KP", PARAM_TYPE_FLOAT); // Roll control proportional gain
	param_add("ATT_ROLL_KI", PARAM_TYPE_FLOAT); // Roll control integral gain

	// Takeoff
	param_add("TKO_ALT", PARAM_TYPE_FLOAT); // Altitude to detect when takeoff is complete, m
	param_add("TKO_PTCH", PARAM_TYPE_FLOAT); // Constant pitch hold during takeoff, deg
	param_add("TKO_ROLL_LIM", PARAM_TYPE_FLOAT); // Roll limit during takeoff, deg

	// Landing
	param_add("LND_FLARE_ALT", PARAM_TYPE_FLOAT); // Altitude to start flare, m
	param_add("LND_FLARE_SINK", PARAM_TYPE_FLOAT); // Target sink rate for flare, m/s

	// TECS
	param_add("TECS_SPD_CRUISE", PARAM_TYPE_FLOAT); // Cruise speed, m/s
	param_add("TECS_SPD_LND", PARAM_TYPE_FLOAT); // Landing approach speed, m/s
	param_add("TECS_MIN_SPD", PARAM_TYPE_FLOAT); // Minimum allowed airspeed, m/s
	param_add("TECS_MAX_SPD", PARAM_TYPE_FLOAT); // Maximum allowed airspeed, m/s
	param_add("TECS_THR_KP", PARAM_TYPE_FLOAT); // Total energy control proportional gain
	param_add("TECS_THR_KI", PARAM_TYPE_FLOAT); // Total energy control integral gain
	param_add("TECS_PTCH_KP", PARAM_TYPE_FLOAT); // Energy balance proportional gain
	param_add("TECS_PTCH_KI", PARAM_TYPE_FLOAT); // Energy balance integral gain
	param_add("TECS_PTCH_LIM", PARAM_TYPE_FLOAT); // Maximum pitch angle, deg
	param_add("TECS_THR_CRUISE", PARAM_TYPE_FLOAT); // Steady-state cruise throttle

	// L1 Controller
	param_add("L1_PERIOD", PARAM_TYPE_FLOAT); // L1 controller period
	param_add("L1_ROLL_LIM", PARAM_TYPE_FLOAT); // L1 controller roll limit

	// Navigator
	param_add("NAV_ACC_RAD", PARAM_TYPE_FLOAT); // Waypoint acceptance radius, m

	// Mixer
	param_add("PWM_MIN_ELE", PARAM_TYPE_INT32); // Min duty elevator, us
	param_add("PWM_MIN_RUD", PARAM_TYPE_INT32); // Min duty rudder, us
	param_add("PWM_MIN_THR", PARAM_TYPE_INT32); // Min duty throttle, us
	param_add("PWM_MIN_AUX1", PARAM_TYPE_INT32); // Min duty auxiliary channel 1, us
	param_add("PWM_MIN_AUX2", PARAM_TYPE_INT32); // Min duty auxiliary channel 2, us
	param_add("PWM_MIN_AUX3", PARAM_TYPE_INT32); // Min duty auxiliary channel 3, us
	param_add("PWM_MAX_ELE", PARAM_TYPE_INT32); // Max duty elevator, us
	param_add("PWM_MAX_RUD", PARAM_TYPE_INT32); // Max duty rudder, us
	param_add("PWM_MAX_THR", PARAM_TYPE_INT32); // Max duty throttle, us
	param_add("PWM_MAX_AUX1", PARAM_TYPE_INT32); // Max duty auxiliary channel 1, us
	param_add("PWM_MAX_AUX2", PARAM_TYPE_INT32); // Max duty auxiliary channel 2, us
	param_add("PWM_MAX_AUX3", PARAM_TYPE_INT32); // Max duty auxiliary channel 3, us
	param_add("PWM_REV_ELE", PARAM_TYPE_INT32); // Reverse elevator channel
	param_add("PWM_REV_RUD", PARAM_TYPE_INT32); // Reverse rudder channel
	param_add("PWM_REV_THR", PARAM_TYPE_INT32); // Reverse throttle channel
	param_add("PWM_REV_AUX1", PARAM_TYPE_INT32); // Reverse auxiliary channel 1
	param_add("PWM_REV_AUX2", PARAM_TYPE_INT32); // Reverse auxiliary channel 2
	param_add("PWM_REV_AUX3", PARAM_TYPE_INT32); // Reverse auxiliary channel 3

	// RC transmitter input
	param_add("RC_MAX_DUTY", PARAM_TYPE_INT32); // Stick input max duty, us
	param_add("RC_MIN_DUTY", PARAM_TYPE_INT32); // Stick input min duty, us

	// AHRS
	param_add("AHRS_BETA_GAIN", PARAM_TYPE_FLOAT); // Madgwick filter beta gain
	param_add("AHRS_MAG_DECL", PARAM_TYPE_FLOAT); // Magnetic declination, deg
	param_add("AHRS_ACC_MAX", PARAM_TYPE_FLOAT); // Max acceleration to enable sensor fusion, g
	param_add("AHRS_HI_X", PARAM_TYPE_FLOAT); // Magnetometer hard-iron calibration
	param_add("AHRS_HI_Y", PARAM_TYPE_FLOAT);
	param_add("AHRS_HI_Z", PARAM_TYPE_FLOAT);
	param_add("AHRS_SI_XX", PARAM_TYPE_FLOAT); // Magnetometer soft-iron calibration
	param_add("AHRS_SI_XY", PARAM_TYPE_FLOAT);
	param_add("AHRS_SI_XZ", PARAM_TYPE_FLOAT);
	param_add("AHRS_SI_YX", PARAM_TYPE_FLOAT);
	param_add("AHRS_SI_YY", PARAM_TYPE_FLOAT);
	param_add("AHRS_SI_YZ", PARAM_TYPE_FLOAT);
	param_add("AHRS_SI_ZX", PARAM_TYPE_FLOAT);
	param_add("AHRS_SI_ZY", PARAM_TYPE_FLOAT);
	param_add("AHRS_SI_ZZ", PARAM_TYPE_FLOAT);

	// Kalman filter
	param_add("EKF_BARO_VAR", PARAM_TYPE_FLOAT); // Barometer variance
	param_add("EKF_GNSS_VAR", PARAM_TYPE_FLOAT); // GNSS variance
	param_add("EKF_OF_ENABLE", PARAM_TYPE_INT32); // Enable optical flow sensor
	param_add("EKF_OF_MIN", PARAM_TYPE_INT32); // Minimum accepted reading, pixels/sec
	param_add("EKF_OF_MAX", PARAM_TYPE_INT32); // Maximum accepted reading, pixels/sec
}
