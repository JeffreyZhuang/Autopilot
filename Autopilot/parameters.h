#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdint.h>

// Airspeed
constexpr float AIRSPEED_CRUISE = 18; // Meters per second
constexpr float AIRSPEED_LANDING = 15;
constexpr float TECS_MIN_SPD_MPS = 10;
constexpr float TECS_MAX_SPD_MPS = 25;

// Throttle
constexpr float TRIM_THROTTLE = 0.1; // Steady-state cruise throttle

// Attitude
constexpr float PTCH_LIM_DEG = 15; // Maximum pitch in either direction
constexpr float ROLL_LIM_DEG = 20; // Maximum roll angle in either direction

// Autoland
constexpr float LAND_GS_DEG = 6; // Landing glideslope angle
constexpr float LAND_FLARE_ALT = 3; // Flare altitude
constexpr float FLARE_SINK_RATE = 0.3;
constexpr float FLARE_TRANS_SEC = 1; // Time to transition to flare
constexpr float TOUCHDOWN_SPD_THR = 1; // Detect touchdown when speed below this value in meters per second

// Takeoff
constexpr float TAKEOFF_ALT = 5; // Altitude that the plane will climb to during takeoff meters per second
constexpr float TAKEOFF_THR = 1; // Throttle set during takeoff between 0 and 1
constexpr float TAKEOFF_PTCH = 10; // Pitch during takeoff

// Guidance
constexpr float MIN_DIST_WP = 50; // Distance in meters from waypoint until switching to next, "radius of acceptance"

// AHRS
constexpr float AHRS_BETA = 0.01; // Filter tuning gain
constexpr float AHRS_ACC_MAX = 2; // Minimum acceleration in inertial frame (g) for accelerometer fusion
constexpr float MAG_DECL = -10.2; // Degrees, determined from online calculator
constexpr float HARD_IRON[3] = {-46.301146, 3.866545, -71.601346};
constexpr float SOFT_IRON[3][3] = {{1.189985, 0.015110, -0.066520},
						  	  	   {0.015110, 1.205787, -0.039344},
								   {-0.066520, -0.039344, 1.183604}};

// Servos
constexpr uint16_t ELEVATOR_MIN_DUTY = 500; // Duty cycle in us
constexpr uint16_t ELEVATOR_MAX_DUTY = 2500;
constexpr uint16_t AILERON_MIN_DUTY = 500;
constexpr uint16_t AILERON_MAX_DUTY = 2500;
constexpr uint16_t THROTTLE_MIN_DUTY = 1000;
constexpr uint16_t THROTTLE_MAX_DUTY = 2000;
constexpr bool REVERSE_ELEVATOR = false;
constexpr bool REVERSE_AILERON = false;

// PID
// Need different gains for each type of vehicle
constexpr float PTCH_KP = 0.04;
constexpr float ROLL_KP = 0.04;
constexpr float THR_KP = 0.01;

// Kalman filter
constexpr float BARO_R = 10000;
constexpr float GNSS_R = 1000000;
constexpr float GNSS_ALT_R = 10000000;

// RC Transmitter
constexpr uint16_t RC_IN_MAX = 1900; // Make sure values are INSIDE the range of radio, NEVER outside
constexpr uint16_t RC_IN_MIN = 1100;

// Store in micro-SD but can be updated from GCS
struct Parameters
{
	// Flight characteristics
	float aspd_cruise;
	float aspd_land;
	float tecs_min_aspd_mps;
	float tecs_max_aspd_mps;
	float throttle_cruise;
	float ptch_lim_deg;
	float roll_lim_deg;
	float min_dist_wp;

	// Landing
	float land_gs_deg;
	float land_flare_alt;
	float flare_sink_rate;
	float flare_trans_sec;
	float touchdown_aspd_thresh;

	// Takeoff
	float takeoff_alt;
	float takeoff_thr;
	float takeof_ptch;

	// Mixer
	uint16_t max_duty[6];
	uint16_t min_duty[6];
	uint16_t rc_in_max;
	uint16_t rc_in_min;
	bool rev_elevator;
	bool rev_aileron;

	// PID
	float ptch_kp;
	float roll_kp;
	float thr_kp;

	// AHRS
	float ahrs_beta;
	float mag_decl;
	float ahrs_acc_max;
	float hard_iron[3];
	float soft_iron[3][3];

	// Kalman filter
	float baro_r;
	float gnss_r;
	float gnss_alt_r;
};

extern const Parameters* Params;

void initializeParameters();

#endif /* PARAMETERS_H_ */
