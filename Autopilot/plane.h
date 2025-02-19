#ifndef PLANE_H_
#define PLANE_H_

#include <stdint.h>

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
	TOUCHDOWN = 9
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
    // State machine
    SystemMode systemMode;
    ManualMode manualMode;
    AutoMode autoMode;
    uint8_t mode_id;

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
    float ahrs_roll = 0;
    float ahrs_pitch = 0;
    float ahrs_yaw = 0;
    uint64_t ahrs_timestamp = 0;

    // Rangefinder
    float rangefinder_dist = 0;
    uint64_t rangefinder_timestamp = 0;

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
    double home_lat = 0;
	double home_lon = 0;
    uint64_t nav_timestamp = 0;

    // Guidance
    uint8_t waypoint_index = 0;
    uint8_t num_waypoints = 0;
	Waypoint waypoints[100]; // 100 max waypoints
    float guidance_hdg_setpoint = 0;
    float guidance_d_setpoint = 0;

    // Landing
    float land_hdg = 0;
    float land_lat = 0;
    float land_lon = 0;
    uint64_t flare_start_time = 0;
    float flare_alt = 0;

    // RC
    float rc_rudder = 0;
    float rc_elevator = 0;
    float rc_throttle = 0;
    bool manual_sw;
    bool mode_sw;
    uint16_t rc_channels[6];

    // Servos
    float aileron_setpoint = 0; // Aileron command from -1 to 1
    float elevator_setpoint = 0;
    float throttle_setpoint = 0;

    // TECS
    float tecs_error_total = 0;
    float tecs_error_diff = 0;

    // Control
    float pitch_setpoint = 0;
    float roll_setpoint = 0;
};

#endif /* PLANE_H_ */
