#ifndef PLANE_H_
#define PLANE_H_

#include <stdint.h>

/**
 * @brief Flight state for state machine
 *
 */
enum class FlightState {
    TAKEOFF = 1,
    CRUISE = 2,
    LAND = 3,
	STABALIZE = 4,
	MANUAL = 5
};

/**
 * @brief All flight data and settings that get passed to classes
 */
struct Plane {
    // Constraints
    float airspeed_min;
    float airspeed_max;
    float airspeed_cruise;
    float pitch_limit_max;
    float pitch_limit_min;
    float roll_limit;

    // Settings
    bool use_compass = true;

    // Monitor
    float batt_current;
    float batt_voltage;
    float autopilot_current;
    float autopilot_voltage;

    // State machine
    FlightState flightState = FlightState::TAKEOFF;

    // Time
    uint64_t time;

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
    float baro_offset = -177.2;
    float baro_temp;
    uint64_t baro_timestamp;

    // GNSS
    float gnss_lat;
    float gnss_lon;
    float gnss_asl;
    int gnss_sats;
    uint64_t gnss_timestamp;

    // AHRS
    float ahrs_roll;
    float ahrs_pitch;
    float ahrs_yaw;
    float ahrs_q0;
    float ahrs_q1;
    float ahrs_q2;
    float ahrs_q3;
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
};

#endif /* PLANE_H_ */
