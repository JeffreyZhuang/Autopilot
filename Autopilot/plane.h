#ifndef PLANE_H_
#define PLANE_H_

#include <stdint.h>

/**
 * @brief Centralized flight data and settings container
 *
 * This class serves as the main data repository for all flight-related
 * information including sensor data, state estimates, control commands,
 * and system state.
 */
class Plane
{
public:
	struct Subscription_handle
	{
		uint64_t timestamp = 0;
	};

	template<typename T>
	class DataHandler
	{
	public:
		bool check_new(const Subscription_handle& subscription_handle) const
		{
			return data.timestamp > subscription_handle.timestamp;
		}

		T get(Subscription_handle& subscription_handle) const
		{
			subscription_handle.timestamp = data.timestamp;
			return data;
		}

		void set(const T& new_data)
		{
			data = new_data;
		}// Add copy? Bypass subscription handle

		const T& get_ref() const { return data; }

	private:
		T data;
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

	struct Pos_est_data
	{
		bool converged = false;
		float pos_n = 0;
		float pos_e = 0;
		float pos_d = 0;
		float vel_n = 0;
		float vel_e = 0;
		float vel_d = 0;
		float gnd_spd = 0;
		float terr_hgt = 0;
		uint64_t timestamp = 0;
	};

	struct Power_data
	{
		float batt_current = 0;
		float batt_voltage = 0;
		float autopilot_current = 0;
		float autopilot_voltage = 0;
		uint64_t timestamp = 0;
	};

	struct Waypoint
	{
		double lat;
		double lon;
		float alt;
	};

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

	// Position estimator
    float baro_offset = 0;

    // Navigator
    uint8_t waypoint_index = 1; // Current waypoint, skip home waypoint
    uint8_t num_waypoints = 0;
	Waypoint waypoints[100]; // 100 max waypoints

	// L1 controller
	struct L1_d_setpoint_data
	{
		float guidance_d_setpoint = 0;
		uint64_t timestamp = 0;
	};

    // RC Transmitter
    bool tx_connected = false;
    float rc_ail_norm = 0; // Normalized RC transmitter stick input, -1 to 1
    float rc_ele_norm = 0;
    float rc_rud_norm = 0;
    float rc_thr_norm = 0;
    bool rc_man_sw = false;
    bool rc_mod_sw = false;

    struct Ctrl_cmd_data
    {
		float rud_cmd = 0; // Control commands from -1 to 1
		float ele_cmd = 0;
		float thr_cmd = 0;
		uint64_t timestamp = 0;
    };

    struct Attitude_setpoint_data
    {
		float pitch_setpoint = 0;
		float roll_setpoint = 0;
		uint64_t timestamp = 0;
    };

    struct Telem_status_data
    {
    	bool waypoints_loaded = false;
    	uint64_t timestamp = 0;
    };

    DataHandler<IMU_data> imu_data;
    DataHandler<Mag_data> mag_data;
    DataHandler<Baro_data> baro_data;
    DataHandler<GNSS_data> gnss_data;
    DataHandler<OF_data> of_data;
    DataHandler<AHRS_data> ahrs_data;
    DataHandler<Pos_est_data> pos_est_data;
    DataHandler<Power_data> power_data;
    DataHandler<Attitude_setpoint_data> attitude_setpoint_data;
    DataHandler<Telem_status_data> telem_status_data;
    DataHandler<Ctrl_cmd_data> ctrl_cmd_data;
    DataHandler<L1_d_setpoint_data> l1_d_setpoint_data;
};

#endif /* PLANE_H_ */
