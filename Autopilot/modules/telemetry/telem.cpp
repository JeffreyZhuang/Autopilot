#include "modules/telemetry/telem.h"

// How to notify all other classes that waypoints have been loaded?

Telem::Telem(HAL* hal, DataBus* data_bus)
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
	  _uncal_imu_sub(data_bus->uncalibrated_imu_node),
	  _uncal_mag_sub(data_bus->uncalibrated_mag_node)
{
	telem_msg.start_reading = false;
	telem_msg.packet_idx = 0;
}

void Telem::update()
{
	_modes_data = _modes_sub.get();
	_ahrs_data = _ahrs_sub.get();
	_gnss_data = _gnss_sub.get();

	send_telemetry();

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
		else if (telem_msg.msg_id == MISSION_ITEM_MSG_ID)
		{
			update_waypoint();
		}
		else if (telem_msg.msg_id == SET_ALTITUDE_MSG_ID)
		{
			update_set_altitude();
		}
		else if (telem_msg.msg_id == REQUEST_CAL_SENSORS_MSG_ID)
		{
			send_calibration();
		}
	}
}

void Telem::send_telemetry()
{
	float current_time_s =_hal->get_time_us() * US_TO_S;

	if (current_time_s - last_vehicle_status_full_transmit_s >= VEHICLE_STATUS_FULL_DT)
	{
		last_vehicle_status_full_transmit_s = current_time_s;

		aplink_vehicle_status_full vehicle_status_full{};
		vehicle_status_full.roll = (int16_t)(_ahrs_data.roll * 100);
		vehicle_status_full.pitch = (int16_t)(_ahrs_data.pitch * 100);
		vehicle_status_full.yaw = (int16_t)(_ahrs_data.yaw * 100);
		vehicle_status_full.alt = 1;
		vehicle_status_full.spd = 2;
		vehicle_status_full.mode_id = get_mode_id();

		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_vehicle_status_full_pack(vehicle_status_full, packet);
		_hal->transmit_telem(packet, len);
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
	uncalibrated_imu_s imu = _uncal_imu_sub.get();
	uncalibrated_mag_s mag = _uncal_mag_sub.get();

	aplink_cal_sensors cal_sensors;
	cal_sensors.ax = imu.ax;
	cal_sensors.ay = imu.ay;
	cal_sensors.az = imu.az;
	cal_sensors.gx = imu.gx;
	cal_sensors.gy = imu.gy;
	cal_sensors.gz = imu.gz;
	cal_sensors.mx = mag.mx;
	cal_sensors.my = mag.my;
	cal_sensors.mz = mag.mz;

	uint8_t packet[MAX_PACKET_LEN];
	uint16_t len = aplink_cal_sensors_pack(cal_sensors, packet);
	_hal->transmit_telem(packet, len);
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
			 param_set.type == APLINK_PARAM_TYPE::APLINK_PARAM_TYPE_FLOAT)
	{
		float value;
		memcpy(&value, param_set.value, sizeof(value));
		param_set_float(param, value);
		success = true;
		printf("Telem params set, value: %f\n", value);
	}
	else if (param_get_type(param) == PARAM_TYPE_INT32 &&
			 param_set.type == APLINK_PARAM_TYPE::APLINK_PARAM_TYPE_INT32)
	{
		int32_t value;
		memcpy(&value, param_set.value, sizeof(value));
		param_set_int32(param, value);
		success = true;
		printf("Telem params set, value: %d\n", value);
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
	aplink_waypoints_count waypoints_count{};
	aplink_waypoints_count_unpack(&telem_msg, &waypoints_count);

	// Reset Counter
	_last_waypoint_loaded = 0;

	// Store data
	_mission_data.num_items = waypoints_count.num_waypoints;

	switch (waypoints_count.type)
	{
	case APLINK_MISSION_ITEM_TYPE::APLINK_MISSION_ITEM_TYPE_WAYPOINT:
		_mission_data.mission_type = MISSION_WAYPOINT;
		break;
	case APLINK_MISSION_ITEM_TYPE::APLINK_MISSION_ITEM_TYPE_LOITER:
		_mission_data.mission_type = MISSION_LOITER;
		break;
	case APLINK_MISSION_ITEM_TYPE::APLINK_MISSION_ITEM_TYPE_LAND:
		_mission_data.mission_type = MISSION_LAND;
		break;
	}

	switch (waypoints_count.direction)
	{
	case APLINK_LOITER_DIRECTION::APLINK_LOITER_DIRECTION_LEFT:
		_mission_data.loiter_direction = LOITER_LEFT;
		break;
	case APLINK_LOITER_DIRECTION::APLINK_LOITER_DIRECTION_RIGHT:
		_mission_data.loiter_direction = LOITER_RIGHT;
		break;
	}

	_mission_data.final_leg_dist = waypoints_count.final_leg;
	_mission_data.glideslope_angle = waypoints_count.glideslope;
	_mission_data.loiter_radius = waypoints_count.radius;
	_mission_data.runway_heading = waypoints_count.runway_heading;

	// Request first waypoint
	aplink_request_waypoint req_waypoint = {
		.index = 0
	};

	uint8_t packet[MAX_PACKET_LEN];
	uint16_t len = aplink_request_waypoint_pack(req_waypoint, packet);
	_hal->transmit_telem(packet, len);
}

void Telem::update_waypoint()
{
	aplink_mission_item mission_item{};
	aplink_mission_item_unpack(&telem_msg, &mission_item);

	_mission_data.mission_items[_last_waypoint_loaded++] = mission_item_t {
		.latitude = (double)mission_item.lat * 1E-7,
		.longitude = (double)mission_item.lon * 1E-7,
	};

	// Check if all waypoints have been loaded
	if (_last_waypoint_loaded == _mission_data.num_items - 1)
	{
		// Update mission if waypoints finished loading
		mission_set(_mission_data);

		// Send acknowledgement
		aplink_waypoints_ack waypoints_ack;
		waypoints_ack.success = true;

		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_waypoints_ack_pack(waypoints_ack, packet);
		_hal->transmit_telem(packet, len);
	}
	else
	{
		// Request next waypoint if waypoints not finished loading
		aplink_request_waypoint req_waypoint;
		req_waypoint.index = ++_last_waypoint_loaded;

		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_request_waypoint_pack(req_waypoint, packet);
		_hal->transmit_telem(packet, len);
	}
}

void Telem::update_set_altitude()
{
	aplink_set_altitude msg{};
	aplink_set_altitude_unpack(&telem_msg, &msg);

	mission_set_altitude(msg.altitude);

	aplink_set_altitude_result result = {
		.success = true
	};

	uint8_t packet[MAX_PACKET_LEN];
	uint16_t len = aplink_set_altitude_result_pack(result, packet);
	_hal->transmit_telem(packet, len);
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

uint8_t Telem::get_mode_id()
{
	// Handle system modes first
	switch (_modes_data.system_mode)
	{
	case System_mode::LOAD_PARAMS:
		return APLINK_MODE_ID::APLINK_MODE_ID_CONFIG;
	case System_mode::STARTUP:
		return APLINK_MODE_ID::APLINK_MODE_ID_STARTUP;
	case System_mode::FLIGHT:
		break;
	default:
		return APLINK_MODE_ID::APLINK_MODE_ID_UNKNOWN;
	}

	// Handle flight modes
	switch (_modes_data.flight_mode)
	{
	case Flight_mode::MANUAL:
		// Handle manual sub-modes
		switch (_modes_data.manual_mode)
		{
		case Manual_mode::DIRECT:
			return APLINK_MODE_ID::APLINK_MODE_ID_MANUAL;
		case Manual_mode::STABILIZED:
			return APLINK_MODE_ID::APLINK_MODE_ID_FBW;
		default:
			return APLINK_MODE_ID::APLINK_MODE_ID_UNKNOWN;
		}
	case Flight_mode::AUTO:
		// Handle auto sub-modes
		switch (_modes_data.auto_mode)
		{
		case Auto_mode::TAKEOFF:
			return APLINK_MODE_ID::APLINK_MODE_ID_TAKEOFF;
		case Auto_mode::MISSION:
			return APLINK_MODE_ID::APLINK_MODE_ID_MISSION;
//		case Auto_mode::LAND:
//			return APLINK_MODE_ID::APLINK_MODE_ID_LAND;
//		case Auto_mode::FLARE:
//			return APLINK_MODE_ID::APLINK_MODE_ID_FLARE;
		default:
			return APLINK_MODE_ID::APLINK_MODE_ID_UNKNOWN;
		}
	default:
		return APLINK_MODE_ID::APLINK_MODE_ID_UNKNOWN;
	}
}
