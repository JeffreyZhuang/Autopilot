#ifndef PLANE_H_
#define PLANE_H_

#include <stdint.h>
#include "parameters.h"

/**
 * The numbers indicate the Mode ID sent to GCS
 * Make sure elements from all enums have different values
 * Cross reference with documentation
 */
enum class SystemMode
{
	BOOT = 0,
	FLIGHT = 1,
};

enum class ManualMode
{
	MANUAL = 2,
	STABILIZED = 3,
};

enum class AutoMode
{
	READY = 4,
	TAKEOFF = 5,
	MISSION = 6,
	LAND = 7,
	FLARE = 8,
	SAFE = 9
};

struct Waypoint {
	double lat;
	double lon;
	double alt;
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
    SystemMode systemMode;
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
    float baro_alt = 0;
    float baro_offset = 0;
    float baro_temp = 0;
    uint64_t baro_timestamp;

    // GNSS
    double gnss_lat = 0;
    double gnss_lon = 0;
    float gnss_asl = 0;
    uint8_t gnss_sats = 0;
    bool fix_quality = 0;
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

    // Rangefinder
    float rangefinder_dist;
    uint64_t rangefinder_timestamp;

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
    double center_lat = 0;
	double center_lon = 0;
    uint64_t nav_timestamp;

    // Guidance
    uint8_t waypoint_index = 0;
    uint8_t num_waypoints = 0;
	Waypoint waypoints[MAX_NUM_WPTS];
    float guidance_hdg_setpoint = 0;
    float guidance_d_setpoint = 0;

    // Landing
    float land_hdg = 0;
    float land_lat = 0;
    float land_lon = 0;
    uint64_t flare_start_time;
    float flare_alt = 0;

    // RC
    float rc_rudder = 0;
    float rc_elevator = 0;
    float rc_throttle = 0;
    bool manual_sw;
    bool mode_sw;

    // Telemetry
    uint8_t latest_packet[TELEM_PKT_LEN];
    uint8_t mode_id;
};

#endif /* PLANE_H_ */
