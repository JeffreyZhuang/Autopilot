#ifndef PLANE_H_
#define PLANE_H_

#include <stdint.h>

/**
 * @brief Flight state for state machine
 *
 */
enum class FlightState
{
	STARTUP = 1,
	TAKEOFF_DETECT = 2,
    TAKEOFF = 3,
    CRUISE = 4,
    LAND = 5,
	STABALIZE = 6,
	MANUAL = 7
};

/**
 * @brief All flight data and settings that get passed to classes
 */
struct Plane
{
    // Constraints
    float airspeed_min;
    float airspeed_max;
    float airspeed_cruise;
    float pitch_limit_max;
    float pitch_limit_min;
    float roll_limit;

    // Monitor
    float batt_current;
    float batt_voltage;
    float autopilot_current;
    float autopilot_voltage;

    // State machine
    FlightState flightState = FlightState::TAKEOFF;

    // Time
    uint64_t time;
    uint64_t loop_execution_time;
    uint32_t loop_iteration = 0;

    // IMU
    float imu_ax;
    float imu_ay;
    float imu_az;
    float imu_gx;
    float imu_gy;
    float imu_gz;
    float imu_temp;
    uint32_t imu_timestamp;

    // Compass
    float compass_mx;
    float compass_my;
    float compass_mz;
    uint64_t compass_timestamp;

    // Barometer
    float baro_alt;
    float baro_offset;
    float baro_temp;
    uint64_t baro_timestamp;

    // GNSS
    float gnss_lat;
    float gnss_lon;
    float gnss_asl;
    int gnss_sats;
    float gnss_center_lat;
    float gnss_center_lon;
    bool gnss_lock = false;
    uint64_t gnss_timestamp;

    // AHRS
    float ahrs_roll;
    float ahrs_pitch;
    float ahrs_yaw;
    float ahrs_q0;
    float ahrs_q1;
    float ahrs_q2;
    float ahrs_q3;
    float ahrs_gain = 10;
    uint64_t ahrs_timestamp;

    // Navigation
    float nav_pos_north;
    float nav_pos_east;
    float nav_pos_down;
    float nav_vel_north;
    float nav_vel_east;
    float nav_vel_down;
    float nav_acc_north;
    float nav_acc_east;
    float nav_acc_down;
    uint64_t nav_timestamp;

    // Guidance
    float guidance_n_setpoint;
    float guidance_e_setpoint;
    float guidance_d_setpoint;

    // RC
    float rc_rudder;
    float rc_elevator;
    float rc_throttle;
    bool rc_switch = false;
};

#endif /* PLANE_H_ */
