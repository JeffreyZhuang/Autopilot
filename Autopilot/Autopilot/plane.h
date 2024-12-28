#ifndef PLANE_H_
#define PLANE_H_

#include <stdint.h>

/**
 * @brief Flight state for state machine
 *
 */
enum class FlightState {
    TAKEOFF = 1,
    LAND = 2
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
    FlightState state = FlightState::TAKEOFF;

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
    uint32_t compass_timestamp;

    // Barometer
    float baro_alt;
    float baro_temp;
    uint32_t baro_timestamp;

    // AHRS
    float ahrs_roll;
    float ahrs_pitch;
    float ahrs_yaw;
    uint32_t ahrs_timestamp;
};

#endif /* PLANE_H_ */
