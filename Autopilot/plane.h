#ifndef PLANE_H_
#define PLANE_H_

#include <stdint.h>

enum class System_mode
{
	CONFIG,
	STARTUP,
	FLIGHT
};

enum class Flight_mode
{
	AUTO,
	MANUAL
};

enum class Auto_mode
{
	TAKEOFF,
	MISSION,
	LAND,
	FLARE,
	TOUCHDOWN
};

enum class Manual_mode
{
	DIRECT,
	STABILIZED
};

struct Waypoint {
	float lat;
	float lon;
	float alt;
};

/**
 * @brief All flight data and settings that get passed to classes
 */
struct Plane
{
    // State machine
	System_mode system_mode;
    Flight_mode flight_mode;
    Auto_mode auto_mode;
    Manual_mode manual_mode;

    // Time
    uint64_t time;
    uint64_t loop_execution_time;
    uint32_t loop_iteration = 0;

    // Power Monitor
	float batt_current = 0;
	float batt_voltage = 0;
	float autopilot_current = 0;
	float autopilot_voltage = 0;

    // IMU
    float imu_ax = 0;
    float imu_ay = 0;
    float imu_az = 0;
    float imu_gx = 0;
    float imu_gy = 0;
    float imu_gz = 0;
    float imu_temp = 0;
    uint32_t imu_timestamp = 0;

    // Compass
    float compass_mx = 0;
    float compass_my = 0;
    float compass_mz = 0;
    uint64_t compass_timestamp = 0;

    // Barometer
    float baro_alt = 0;
    float baro_offset = 0;
    float baro_temp = 0;
    uint64_t baro_timestamp = 0;

    // GNSS
    double gnss_lat = 0;
    double gnss_lon = 0;
    float gnss_asl = 0;
    uint8_t gnss_sats = 0;
    bool gps_fix = false;
    uint64_t gnss_timestamp = 0;

    // AHRS
    bool ahrs_converged = false;
    float ahrs_roll = 0;
    float ahrs_pitch = 0;
    float ahrs_yaw = 0;
    uint64_t ahrs_timestamp = 0;

    // Optical flow
    int16_t of_x = 0;
    int16_t of_y = 0;
    uint64_t of_timestamp = 0;

    // Navigation
    bool nav_converged = false;
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
    float home_lat = 0;
	float home_lon = 0;
    uint64_t nav_timestamp = 0;

    // Guidance
    uint8_t waypoint_index = 1; // Skip home waypoint
    uint8_t num_waypoints = 0;
	Waypoint waypoints[100]; // 100 max waypoints
    float guidance_hdg_setpoint = 0;
    float guidance_d_setpoint = 0;

    // RC Transmitter
    bool tx_connected = false;
    float rc_ail_norm = 0; // Normalized RC transmitter stick input, -1 to 1
    float rc_ele_norm = 0;
    float rc_rud_norm = 0;
    float rc_thr_norm = 0;
    bool rc_man_sw = false;
    bool rc_mod_sw = false;

    // Control commands from -1 to 1
    float aileron_setpoint = 0;
    float elevator_setpoint = 0;
    float throttle_setpoint = 0;

    // TECS
    float tecs_energy_total_setpoint = 0;
	float tecs_energy_total = 0;
	float tecs_energy_diff_setpoint = 0;
	float tecs_energy_diff = 0;

    // Control
    float pitch_setpoint = 0;
    float roll_setpoint = 0;

    bool waypoints_loaded = false;
};

#endif /* PLANE_H_ */
