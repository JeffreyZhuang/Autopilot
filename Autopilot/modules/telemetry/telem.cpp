#include "modules/telemetry/telem.h"

Telem::Telem(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _pos_est_sub(data_bus->pos_est_node),
	  _ahrs_sub(data_bus->ahrs_node),
	  _gnss_sub(data_bus->gnss_node),
	  _modes_sub(data_bus->modes_node),
	  _l1_sub(data_bus->l1_node),
	  _navigator_sub(data_bus->navigator_node),
	  _power_sub(data_bus->power_node),
	  _tecs_sub(data_bus->tecs_node),
	  _ctrl_cmd_sub(data_bus->ctrl_cmd_node),
	  _baro_sub(data_bus->baro_node),
	  _imu_sub(data_bus->imu_node),
	  _telem_pub(data_bus->telem_node)
{
}

void Telem::update()
{
	if (_modes_data.system_mode == System_mode::CALIBRATION)
	{
		update_calibration();
	}
	else if (_modes_data.system_mode == System_mode::DOWNLOAD_LOGS)
	{
		update_download_logs();
	}
	else
	{
		switch (_telem_state)
		{
		case TelemState::LOAD_PARAMS:
			update_load_params();
			break;
		case TelemState::LOAD_WAYPOINTS:
			update_load_waypoints();
			break;
		case TelemState::SEND_TELEMETRY:
			update_send_telemetry();
			break;
		}
	}
}

void Telem::update_load_params()
{
	if (read_telem(&telem_msg))
	{
		if (telem_msg.msg_id == PARAM_SET_MSG_ID &&
			telem_msg.payload_len == sizeof(aplink_param_set))
		{
			aplink_param_set param_set;
			aplink_param_set_decode(&telem_msg, &param_set);

			param_t param = param_find(param_set.param_id);
			if (param == PARAM_INVALID)
			{
				printf("Param invalid\n");
			}
			else if (param_get_type(param) == PARAM_TYPE_FLOAT)
			{
				param_set_float(param, param_set.f);
				printf("Telem params set\n");
			}
			else if (param_get_type(param) == PARAM_TYPE_INT32)
			{
				param_set_int32(param, param_set.i);
				printf("Telem params set\n");
			}
		}
	}

	if (param_all_set())
	{
		_telem_state = TelemState::LOAD_WAYPOINTS;
	}
}

void Telem::update_load_waypoints()
{
	if (read_telem(&telem_msg))
	{
		if (telem_msg.msg_id == WAYPOINT_MSG_ID &&
			telem_msg.payload_len == sizeof(aplink_waypoint))
		{
			aplink_waypoint waypoint_payload;
			aplink_waypoint_msg_decode(&telem_msg, &waypoint_payload);

			if (waypoint_payload.waypoint_index == 0)
			{
				_telem_data.num_waypoints = 0; // Reset if its the first waypoint
			}

			if (waypoint_payload.waypoint_index == _telem_data.num_waypoints)
			{
				_telem_data.waypoints[_telem_data.num_waypoints++] = Waypoint{
					(double)waypoint_payload.lat * 1E-7, (double)waypoint_payload.lon * 1E-7,
					(float)waypoint_payload.alt * 1E-1f
				};

				if (waypoint_payload.waypoint_index == _telem_data.num_waypoints - 1)
				{
					_telem_data.waypoints_loaded = true;
					_telem_state = TelemState::SEND_TELEMETRY;
				}

				printf("Telem waypoint set\n");
			}
			else
			{
				printf("Missed waypoint\n");
			}
		}
	}
}

void Telem::update_send_telemetry()
{
	float current_time_s =_hal->get_time_us() * US_TO_S;

	if (current_time_s - last_vfr_hud_transmit_s >= VFR_HUD_DT)
	{
		last_vfr_hud_transmit_s = current_time_s;

		aplink_vfr_hud vfr_hud;
		vfr_hud.roll = (int16_t)(_ahrs_data.roll * 100);
		vfr_hud.pitch = (int16_t)(_ahrs_data.pitch * 100);
		vfr_hud.heading = (uint16_t)(_ahrs_data.yaw * 10);

		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_vfr_hud_pack(vfr_hud, packet);

		_hal->transmit_telem(packet, len);
	}

	if (current_time_s - last_nav_display_transmit_s >= NAV_DISPLAY_DT)
	{
		last_nav_display_transmit_s = current_time_s;

		aplink_nav_display nav_display;
		nav_display.pos_est_north = _pos_est_data.pos_n;
		nav_display.pos_est_east = _pos_est_data.pos_e;

		uint8_t packet[MAX_PACKET_LEN];
		uint16_t len = aplink_nav_display_pack(nav_display, packet);

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
}

void Telem::update_calibration()
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

void Telem::update_download_logs()
{
	// Read from storage sub
}

bool Telem::read_telem(aplink_msg* msg)
{
	while (!_hal->telem_buffer_empty())
	{
		uint8_t byte;
		_hal->read_telem(&byte);

		return aplink_parse_byte(msg, byte);
	}

	return false;
}

// Returns unique state identifier
// Returns 255 if unknown state
uint8_t Telem::get_current_state()
{
    switch (_modes_data.system_mode)
    {
	case System_mode::LOAD_PARAMS:
		return 0;
	case System_mode::STARTUP:
		return 1;
	case System_mode::FLIGHT:
		switch (_modes_data.flight_mode)
		{
		case Flight_mode::AUTO:
			switch (_modes_data.auto_mode)
			{
			case Auto_mode::TAKEOFF:
				return 2;
			case Auto_mode::MISSION:
				return 3;
			case Auto_mode::LAND:
				return 4;
			case Auto_mode::FLARE:
				return 5;
			}
		case Flight_mode::MANUAL:
			switch (_modes_data.manual_mode)
			{
				case Manual_mode::DIRECT:
					return 6;
				case Manual_mode::STABILIZED:
					return 7;
			}
		}
    }

    return 255;
}
