#include "modules/telemetry/telem.h"

// How to notify all other classes that waypoints have been loaded?

Telem::Telem(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _local_pos_sub(data_bus->local_position_node),
	  _ahrs_sub(data_bus->ahrs_node),
	  _gnss_sub(data_bus->gnss_node),
	  _modes_sub(data_bus->modes_node),
	  _position_control_sub(data_bus->position_control_node),
	  _waypoint_sub(data_bus->waypoint_node),
	  _power_sub(data_bus->power_node),
	  _ctrl_cmd_sub(data_bus->ctrl_cmd_node),
	  _baro_sub(data_bus->baro_node),
	  _imu_sub(data_bus->imu_node),
	  _telem_new_waypoint_pub(data_bus->telem_new_waypoint_node)
{
}

void Telem::update()
{
	_modes_data = _modes_sub.get();
	_ahrs_data = _ahrs_sub.get();
	_gnss_data = _gnss_sub.get();

	if (_modes_data.system_mode == System_mode::CALIBRATION)
	{
		send_calibration();
	}
	else
	{
		send_telemetry();
	}

	if (read_telem(&telem_msg))
	{
		if (telem_msg.msg_id == WAYPOINTS_COUNT_MSG_ID)
		{
			update_waypoints_count();
		}
		else if (telem_msg.msg_id == PARAM_SET_MSG_ID &&
				 telem_msg.payload_len == sizeof(aplink_param_set))
		{
			update_param_set();
		}
		else if (telem_msg.msg_id == WAYPOINT_MSG_ID)
		{
			update_waypoint();
		}
	}
}

void Telem::send_telemetry()
{
	float current_time_s =_hal->get_time_us() * US_TO_S;

	if (current_time_s - last_vehicle_status_full_transmit_s >= VEHICLE_STATUS_FULL_DT)
	{
		last_vehicle_status_full_transmit_s = current_time_s;

		printf("roll: %f\n", _ahrs_data.roll);

		aplink_vehicle_status_full vehicle_status_full{};
		vehicle_status_full.roll = (int16_t)(_ahrs_data.roll * 100);
		vehicle_status_full.pitch = (int16_t)(_ahrs_data.pitch * 100);
		vehicle_status_full.yaw = (int16_t)(_ahrs_data.yaw * 10);
		vehicle_status_full.alt = 1;

		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_vehicle_status_full_pack(vehicle_status_full, packet);
		_hal->transmit_telem(packet, len);

		printf("transmit telem\n");
	}

	if (current_time_s - last_gps_raw_transmit_s >= GPS_RAW_DT)
	{
		last_gps_raw_transmit_s = current_time_s;

		aplink_gps_raw gps_raw;
		gps_raw.lat = (int32_t)(_gnss_data.lat * 1E7);
		gps_raw.lon = (int32_t)(_gnss_data.lon * 1E7);
		gps_raw.sats = _gnss_data.sats;
		gps_raw.fix = _gnss_data.fix;

		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_gps_raw_pack(gps_raw, packet);
		_hal->transmit_telem(packet, len);
	}

	if (current_time_s - last_power_transmit_s >= POWER_DT)
	{
		last_power_transmit_s = current_time_s;

		aplink_power power;
		power.ap_curr = 0;
		power.batt_curr = 0;
		power.batt_volt = 0;
		power.batt_used = 0;

		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_power_pack(power, packet);
		_hal->transmit_telem(packet, len);
	}
}

void Telem::send_calibration()
{
	float current_time_s =_hal->get_time_us() * US_TO_S;

	if (current_time_s - last_cal_sensors_transmit_s >= CAL_SENSORS_DT)
	{
		last_cal_sensors_transmit_s = current_time_s;

		aplink_cal_sensors cal_sensors;
		cal_sensors.ax = _imu_data.ax;
		cal_sensors.ay = _imu_data.ay;
		cal_sensors.az = _imu_data.az;
		cal_sensors.gx = _imu_data.gx;
		cal_sensors.gy = _imu_data.gy;
		cal_sensors.gz = _imu_data.gz;

		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_cal_sensors_pack(cal_sensors, packet);

		_hal->transmit_telem(packet, len);
	}
}

void Telem::update_param_set()
{
	aplink_param_set param_set;
	aplink_param_set_unpack(&telem_msg, &param_set);

	printf("Telem param name: %s\n", param_set.name);

	bool success = false;
	param_t param = param_find(param_set.name);
	if (param == PARAM_INVALID)
	{
		printf("Param not found\n");
	}
	else if (param_get_type(param) == PARAM_TYPE_FLOAT &&
			 param_set.type == PARAM_TYPE::FLOAT)
	{
		float value;
		memcpy(&value, param_set.value, sizeof(value));
		param_set_float(param, value);
		success = true;
		printf("Telem params set\n");
	}
	else if (param_get_type(param) == PARAM_TYPE_INT32 &&
			 param_set.type == PARAM_TYPE::INT32)
	{
		int32_t value;
		memcpy(&value, param_set.value, sizeof(value));
		param_set_int32(param, value);
		success = true;
		printf("Telem params set\n");
	}
	else
	{
		printf("Param type wrong\n");
	}

	// Send acknowledgement
	if (success)
	{
		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_param_set_pack(param_set, packet);
		_hal->transmit_telem(packet, len);
	}
}

void Telem::update_waypoints_count()
{
	aplink_waypoints_count waypoints_count;
	aplink_waypoints_count_unpack(&telem_msg, &waypoints_count);

	// Reset Counter
	_last_waypoint_loaded = 0;

	// Store number of waypoints
	_num_waypoints = waypoints_count.num_waypoints;

	// Request first waypoint
	aplink_request_waypoint req_waypoint;
	req_waypoint.index = 0;

	uint8_t packet[MAX_PACKET_LEN];
	uint16_t len = aplink_request_waypoint_pack(req_waypoint, packet);
	_hal->transmit_telem(packet, len);
}

void Telem::update_waypoint()
{
	aplink_waypoint waypoint;
	aplink_waypoint_unpack(&telem_msg, &waypoint);

	// Make sure takeoff and landing waypoint altitudes set to 0
	if ((_last_waypoint_loaded == 0 || _last_waypoint_loaded == _num_waypoints - 1) && waypoint.alt == 0)
	{
		// TODO: Instead of storing waypoints in commander, maybe use a global waypoints storage library like parameters
		telem_new_waypoint_s new_waypoint_s;
		new_waypoint_s.lat = (double)waypoint.lat * 1E-7;
		new_waypoint_s.lon = (double)waypoint.lon * 1E-7;
		new_waypoint_s.alt = -waypoint.alt;
		new_waypoint_s.index = _last_waypoint_loaded;
		new_waypoint_s.num_waypoints = _num_waypoints;
		new_waypoint_s.timestamp = _hal->get_time_us();
		_telem_new_waypoint_pub.publish(new_waypoint_s);

		// Check if all waypoints have been loaded
		if (_last_waypoint_loaded == _num_waypoints - 1)
		{
			// Send acknowledgement
			aplink_waypoints_ack waypoints_ack;
			waypoints_ack.waypoints_loaded = 0;

			uint8_t packet[MAX_PACKET_LEN];
			uint16_t len = aplink_waypoints_ack_pack(waypoints_ack, packet);
			_hal->transmit_telem(packet, len);
		}
		else
		{
			// Request next waypoint
			aplink_request_waypoint req_waypoint;
			req_waypoint.index = ++_last_waypoint_loaded;

			uint8_t packet[MAX_PACKET_LEN];
			uint16_t len = aplink_request_waypoint_pack(req_waypoint, packet);
			_hal->transmit_telem(packet, len);
		}
	}
}

bool Telem::read_telem(aplink_msg* msg)
{
	while (!_hal->telem_buffer_empty())
	{
		uint8_t byte;
		_hal->read_telem(&byte);

		if (aplink_parse_byte(msg, byte))
		{
			return true;
		}
	}

	return false;
}
