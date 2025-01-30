#ifndef PLANE_H_
#define PLANE_H_

#include <stdint.h>
#include "parameters.h"

enum class ManualMode
{
	MANUAL,
	STABILIZED
};

enum class AutoMode
{
	TAKEOFF_DETECT, // Change to ARMED
	TAKEOFF,
	MISSION,
	LAND
};

/**
 * @brief All flight data and settings that get passed to classes
 */
struct Plane
{
    // Monitor
    float batt_current = 0;
    float batt_voltage = 0;
    float autopilot_current = 0;
    float autopilot_voltage = 0;

    // State machine
    ManualMode manualMode;
    AutoMode autoMode;

    // Time
    uint64_t time;
    uint64_t loop_execution_time;
    uint32_t loop_iteration;

    // IMU
    float imu_ax = 0;
    float imu_ay = 0;
    float imu_az = 0;
    float imu_gx = 0;
    float imu_gy = 0;
    float imu_gz = 0;
    float imu_temp;
    uint32_t imu_timestamp;

    // Compass
    float compass_mx = 0;
    float compass_my = 0;
    float compass_mz = 0;
    uint64_t compass_timestamp;

    // Barometer
    float baro_alt;
    float baro_offset;
    float baro_temp;
    uint64_t baro_timestamp;

    // GNSS
    float gnss_lat = 0;
    float gnss_lon = 0;
    float gnss_asl = 0;
    uint8_t gnss_sats = 0;
    float gnss_center_lat = 0;
    float gnss_center_lon = 0;
    bool gnss_lock = false;
    uint64_t gnss_timestamp;

    // AHRS
    float ahrs_roll = 0;
    float ahrs_pitch = 0;
    float ahrs_yaw = 0;
    float ahrs_q0 = 0;
    float ahrs_q1 = 0;
    float ahrs_q2 = 0;
    float ahrs_q3 = 0;
    uint64_t ahrs_timestamp;

    // Navigation
    float nav_pos_north = 0;
    float nav_pos_east = 0;
    float nav_pos_down = 0;
    float nav_vel_north = 0;
    float nav_vel_east = 0;
    float nav_vel_down = 0;
    float nav_acc_north = 0;
    float nav_acc_east = 0;
    float nav_acc_down = 0;
    float nav_airspeed = 0;
    uint64_t nav_timestamp;

    // Guidance
    float guidance_n_setpoint = 0;
    float guidance_e_setpoint = 0;
    float guidance_d_setpoint = 0;

    // RC
    float rc_rudder = 0;
    float rc_elevator = 0;
    float rc_throttle = 0;
    bool rc_switch; // Change to manual_sw
};

#endif /* PLANE_H_ */
