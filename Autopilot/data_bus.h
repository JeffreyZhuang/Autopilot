#ifndef DATA_BUS_H_
#define DATA_BUS_H_

#include "modes.h"
#include <stdint.h>

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

struct local_position_s
{
	uint64_t timestamp = 0;

	// Position in local NED frame
	float x = 0; // North position in NED earth-fixed frame, (meters)
	float y = 0; // East position in NED earth-fixed frame, (meters)
	float z = 0; // Down position (negative altitude) in NED earth-fixed frame, (meters)

	// Velocity in NED frame
	float vx = 0; // North velocity in NED earth-fixed frame, (meters/sec)
	float vy = 0; // East velocity in NED earth-fixed frame, (meters/sec)
	float vz = 0; // Down velocity in NED earth-fixed frame, (meters/sec)

	float gnd_spd = 0;
	float terr_hgt = 0;

	// Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
	bool ref_xy_set = false; // true if position (x, y) has a valid global reference (ref_lat, ref_lon)
	bool ref_z_set = false; // true if z has a valid global reference (ref_alt)
	double ref_lat;
	double ref_lon;
	float ref_alt; // Reference altitude ASL

	bool converged = false;
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
	uint64_t timestamp = 0;
};

struct Telem_data
{
	uint8_t num_waypoints = 0;
	bool waypoints_loaded = false;
	uint64_t timestamp = 0;
};

struct telem_new_waypoint_s
{
	double lat;
	double lon;
	float alt;
	uint8_t index;
	uint8_t num_waypoints;
	uint64_t timestamp;
};

struct position_control_s
{
	float pitch_setpoint;
	float roll_setpoint;
	float throttle_setpoint;
	uint64_t timestamp;
};

struct position_control_status_s
{
	float cross_trk_err;
	float d_setpoint;
	float total_energy_setpoint = 0;
	float total_energy = 0;
	float energy_balance_setpoint = 0;
	float energy_balance = 0;
	uint64_t timestamp;
};

struct Modes_data
{
	System_mode system_mode = System_mode::LOAD_PARAMS;
	Flight_mode flight_mode = Flight_mode::MANUAL;
	Auto_mode auto_mode = Auto_mode::TAKEOFF;
	Manual_mode manual_mode = Manual_mode::DIRECT;
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

struct waypoint_s
{
	float previous_north = 0;
	float previous_east = 0;
	float previous_alt = 0;
	float current_north = 0;
	float current_east = 0;
	float current_alt = 0;
	uint8_t current_index = 0;
	uint64_t timestamp = 0;
};

struct time_s
{
	float dt_s = 0;
	uint32_t loop_iteration = 0;
	uint32_t unix_epoch_time = 0;
	uint64_t timestamp = 0;
};

struct hitl_sensors_s
{
	float imu_ax;
	float imu_ay;
	float imu_az;
	float imu_gx;
	float imu_gy;
	float imu_gz;
	float mag_x;
	float mag_y;
	float mag_z;
	float baro_asl;
	int32_t gps_lat;
	int32_t gps_lon;
	int16_t of_x;
	int16_t of_y;
	uint64_t timestamp = 0;
};

struct HITL_output_data
{
	uint16_t ele_duty = 0;
	uint16_t rud_duty = 0;
	uint16_t thr_duty = 0;
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
	Publisher(Node<T>& node) : node(node) {}

	void publish(const T& new_data)
	{
		node.set(new_data);
	}

private:
	Node<T>& node;
};

template<typename T>
class Subscriber
{
public:
	Subscriber(Node<T>& node) : node(node) {}

	bool check_new()
	{
		return node.check_new(last_timestamp);
	}

	T get()
	{
		return node.get(&last_timestamp);
	}

private:
	Node<T>& node;
	uint64_t last_timestamp = 0;
};

/**
 * @brief Centralized flight data and settings container
 *
 * This class serves as the main data repository for all flight-related
 * information including sensor data, state estimates, control commands,
 * and system state.
 */
struct Data_bus
{
	Node<time_s> time_node;
	Node<Modes_data> modes_node;
    Node<IMU_data> imu_node;
    Node<Mag_data> mag_node;
    Node<Baro_data> baro_node;
    Node<GNSS_data> gnss_node;
    Node<OF_data> of_node;
    Node<AHRS_data> ahrs_node;
    Node<local_position_s> local_position_node;
    Node<Power_data> power_node;
    Node<Telem_data> telem_node;
   	Node<Ctrl_cmd_data> ctrl_cmd_node;
   	Node<RC_data> rc_node;
   	Node<waypoint_s> waypoint_node;
   	Node<hitl_sensors_s> hitl_sensors_node;
   	Node<HITL_output_data> hitl_output_node;
   	Node<telem_new_waypoint_s> telem_new_waypoint_node;
   	Node<position_control_s> position_control_node;
};

#endif /* DATA_BUS_H_ */
