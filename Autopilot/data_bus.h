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

struct TECS_data
{
	float pitch_setpoint = 0;
	uint64_t timestamp = 0;
};

struct Telem_data
{
	bool waypoints_loaded = false;
	uint8_t num_waypoints = 0;
	Waypoint waypoints[100]; // 100 max waypoints
	uint64_t timestamp = 0;
};

struct L1_data
{
	float d_setpoint = 0;
	float roll_setpoint = 0;
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
class Node
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
	Publisher(Node<T>& handler) : handler(handler) {}

	void publish(const T& new_data)
	{
		handler.set(new_data);
	}

private:
	Node<T>& handler;
};

template<typename T>
class Subscriber
{
public:
	Subscriber(Node<T>& handler) : handler(handler) {}

	bool check_new()
	{
		return handler.check_new(last_timestamp);
	}

	T get()
	{
		return handler.get(&last_timestamp);
	}

private:
	Node<T>& handler;
	uint64_t last_timestamp = 0;
};

// INSTEAD OF GET INSTANCE YOU CAN PASS AS POINTER TO ALL CLASSES

/**
 * @brief Centralized flight data and settings container
 *
 * This class serves as the main data repository for all flight-related
 * information including sensor data, state estimates, control commands,
 * and system state.
 */
struct Data_bus
{
	Node<Time_data> time_node;
	Node<Modes_data> modes_node;
    Node<IMU_data> imu_node;
    Node<Mag_data> mag_node;
    Node<Baro_data> baro_node;
    Node<GNSS_data> gnss_node;
    Node<OF_data> of_node;
    Node<AHRS_data> ahrs_node;
    Node<Pos_est_data> pos_est_node;
    Node<Power_data> power_node;
    Node<TECS_data> tecs_node;
    Node<Telem_data> telem_node;
   	Node<Ctrl_cmd_data> ctrl_cmd_node;
   	Node<L1_data> l1_node;
   	Node<RC_data> rc_node;
   	Node<Navigator_data> navigator_node;
};

// Each node has ID integer. Initialize the nodes in data bus in array. Function to get node given ID that does a search through array.
// Then you input Node<Time_data>.ID for time data id, or something

#endif /* DATA_BUS_H_ */
