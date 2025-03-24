#ifndef PLANE_H_
#define PLANE_H_

#include "modes.h"
#include <stdint.h>

struct Waypoint {
	double lat;
	double lon;
	float alt;
};

struct Subscription_handle
{
	uint64_t timestamp = 0;
};

struct IMU_data
{
	float gx = 0;
	float gy = 0;
	float gz = 0;
	float ax = 0;
	float ay = 0;
	float az = 0;
	uint64_t timestamp = 0;
};

struct Mag_data
{
	float x = 0;
	float y = 0;
	float z = 0;
	uint64_t timestamp = 0;
};

struct Baro_data
{
	float alt = 0;
	uint64_t timestamp = 0;
};

struct GNSS_data
{
	double lat = 0;
	double lon = 0;
	float asl = 0;
	uint8_t sats = 0;
	bool fix = false;
	uint64_t timestamp = 0;
};

struct AHRS_data
{
	bool converged = false;
	float roll = 0;
	float pitch = 0;
	float yaw = 0;
	uint64_t timestamp = 0;
};

struct OF_data
{
	int16_t x = 0;
	int16_t y = 0;
	uint64_t timestamp = 0;
};

/**
 * @brief All flight data and settings that get passed to classes
 */
class Plane
{
public:
    // State machine
	System_mode system_mode;
    Flight_mode flight_mode;
    Auto_mode auto_mode;
    Manual_mode manual_mode;

    // Time
    uint64_t time_us = 0;
    float dt_s = 0;
    uint32_t loop_iteration = 0;
    uint64_t us_since_epoch = 0;

    // Power Monitor
	float batt_current = 0;
	float batt_voltage = 0;
	float autopilot_current = 0;
	float autopilot_voltage = 0;

    float baro_offset = 0;

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
    float nav_gnd_spd = 0;
    float nav_terr_hgt = 0;
    float home_lat = 0;
	float home_lon = 0;
    uint64_t nav_timestamp = 0;

    // Navigator
    uint8_t waypoint_index = 1; // Skip home waypoint
    uint8_t num_waypoints = 0;
	Waypoint waypoints[100]; // 100 max waypoints
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
    float rud_cmd = 0;
    float ele_cmd = 0;
    float thr_cmd = 0;

    // TECS
    float tecs_energy_total_setpoint = 0;
	float tecs_energy_total = 0;
	float tecs_energy_diff_setpoint = 0;
	float tecs_energy_diff = 0;

    // Control
    float pitch_setpoint = 0;
    float roll_setpoint = 0;

    bool waypoints_loaded = false;

    bool check_new_imu_data(Subscription_handle subscription_handle);
    IMU_data get_imu_data(Subscription_handle subscription_handle);
    void set_imu_data(IMU_data imu_data);

    bool check_new_mag_data(Subscription_handle subscription_handle);
	Mag_data get_mag_data(Subscription_handle subscription_handle);
	void set_mag_data(Mag_data mag_data);

	bool check_new_baro_data(Subscription_handle subscription_handle);
	Baro_data get_baro_data(Subscription_handle subscription_handle);
	void set_baro_data(Baro_data baro_data);

	bool check_new_gnss_data(Subscription_handle subscription_handle);
	GNSS_data get_gnss_data(Subscription_handle subscription_handle);
	void set_gnss_data(GNSS_data gnss_data);

    bool check_new_of_data(Subscription_handle subscription_handle);
	OF_data get_of_data(Subscription_handle subscription_handle);
	void set_of_data(OF_data of_data);

	bool check_new_ahrs_data(Subscription_handle subscription_handle);
	AHRS_data get_ahrs_data(Subscription_handle subscription_handle);
	void set_ahrs_data(AHRS_data ahrs_data);

private:
    IMU_data _imu_data;
    Mag_data _mag_data;
    Baro_data _baro_data;
    GNSS_data _gnss_data;
    OF_data _of_data;
    AHRS_data _ahrs_data;
};

#endif /* PLANE_H_ */
