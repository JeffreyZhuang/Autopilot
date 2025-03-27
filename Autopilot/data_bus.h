#ifndef DATA_BUS_H_
#define DATA_BUS_H_

#include <stdint.h>

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
	float baro_offset = 0;
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

struct Ctrl_cmd_data
{
	float rud_cmd = 0; // Control commands [-1, 1]
	float ele_cmd = 0;
	float thr_cmd = 0;
	uint64_t timestamp = 0;
};

struct Att_setpoint_data
{
	float pitch_setpoint = 0;
	float roll_setpoint = 0;
	uint64_t timestamp = 0;
};

struct Telem_data
{
	bool waypoints_loaded = false;
	uint8_t num_waypoints = 0;
	Waypoint waypoints[100]; // 100 max waypoints
	uint64_t timestamp = 0;
};

struct L1_d_setpoint_data
{
	float guidance_d_setpoint = 0;
	uint64_t timestamp = 0;
};

struct Modes_data
{
	System_mode system_mode;
	Flight_mode flight_mode;
	Auto_mode auto_mode;
	Manual_mode manual_mode;
	uint64_t timestamp = 0;
};

struct RC_data
{
	bool tx_conn = false;
	float ail_norm = 0; // Normalized RC transmitter stick input, -1 to 1
	float ele_norm = 0;
	float rud_norm = 0;
	float thr_norm = 0;
	bool man_sw = false;
	bool mod_sw = false;
	uint64_t timestamp = 0;
};

struct Navigator_data
{
	uint8_t waypoint_index = 0; // Current waypoint, default 1 to skip home waypoint
	uint64_t timestamp = 0;
};

struct Time_data
{
	float dt_s = 0;
	uint32_t loop_iteration = 0;
	uint64_t us_since_epoch = 0;
	uint64_t timestamp = 0;
};

template<typename T>
class DataHandler
{
public:
	bool check_new(const uint64_t last_timestamp) const
	{
		return data.timestamp > last_timestamp;
	}

	T get(uint64_t *timestamp) const
	{
		*timestamp = data.timestamp;
		return data;
	}

	void set(const T& new_data)
	{
		data = new_data;
	}

private:
	T data;
};

template<typename T>
class Publisher
{
public:
	Publisher(DataHandler<T>& handler) : handler(handler) {}

	void publish(const T& new_data)
	{
		handler.set(new_data);
	}

private:
	DataHandler<T>& handler;
};

template<typename T>
class Subscriber
{
public:
	Subscriber(DataHandler<T>& handler) : handler(handler) {}

	bool check_new()
	{
		return handler.check_new(last_timestamp);
	}

	T get()
	{
		return handler.get(&last_timestamp);
	}

private:
	DataHandler<T>& handler;
	uint64_t last_timestamp = 0;
};

/**
 * @brief Centralized flight data and settings container
 *
 * This class serves as the main data repository for all flight-related
 * information including sensor data, state estimates, control commands,
 * and system state.
 */
class Data_bus
{
public:
	DataHandler<Modes_data> modes_Data;
    DataHandler<IMU_data> imu_data;
    DataHandler<Mag_data> mag_data;
    DataHandler<Baro_data> baro_data;
    DataHandler<GNSS_data> gnss_data;
    DataHandler<OF_data> of_data;
    DataHandler<AHRS_data> ahrs_data;
    DataHandler<Pos_est_data> pos_est_data;
    DataHandler<Power_data> power_data;
    DataHandler<Att_setpoint_data> att_setpoint_data;
    DataHandler<Telem_data> telem_data;
    DataHandler<Ctrl_cmd_data> ctrl_cmd_data;
    DataHandler<L1_d_setpoint_data> l1_d_setpoint_data;
    DataHandler<RC_data> rc_data;
    DataHandler<Navigator_data> navigator_data;

    Waypoint get_home() const
    {
    	// First waypoint is home
    	return telem_data.get(nullptr).waypoints[0];
    }

    static Data_bus& get_instance()
    {
    	static Data_bus instance;
    	return instance;
    }

private:
    Data_bus() = default;
};

#endif /* DATA_BUS_H_ */
