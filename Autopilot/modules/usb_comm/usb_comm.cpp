#include "usb_comm.h"

USBComm::USBComm(HAL* hal, DataBus* data_bus)
	: Module(hal, data_bus),
	  _local_pos_sub(data_bus->local_position_node),
	  _ahrs_sub(data_bus->ahrs_node),
	  _gnss_sub(data_bus->gnss_node),
	  _modes_sub(data_bus->modes_node),
	  _position_control_sub(data_bus->position_control_node),
	  _waypoint_sub(data_bus->waypoint_node),
	  _power_sub(data_bus->power_node),
	  _ctrl_cmd_sub(data_bus->ctrl_cmd_node),
	  _hitl_output_sub(data_bus->hitl_output_node),
	  _baro_sub(data_bus->baro_node),
	  _hitl_sensors_pub(data_bus->hitl_sensors_node)
{
}

void USBComm::update()
{
	_modes_data = _modes_sub.get();
	_ahrs_data = _ahrs_sub.get();
	_gnss_data = _gnss_sub.get();
	_hitl_output_data = _hitl_output_sub.get();

	// Read
	if (read_usb())
	{
		if (msg.msg_id == HITL_SENSORS_MSG_ID)
		{
			read_hitl();
		}
	}

	// Transmit status for debug purposes
	transmit_vehicle_status();
	transmit_hitl();
}

bool USBComm::read_usb()
{
	while (!_hal->usb_buffer_empty())
	{
		uint8_t byte;
		_hal->usb_read(&byte);

		if (aplink_parse_byte(&msg, byte))
		{
			return true;
		}
	}

	return false;
}

void USBComm::read_hitl()
{
	aplink_hitl_sensors hitl_sensors;
	aplink_hitl_sensors_unpack(&msg, &hitl_sensors);

	hitl_sensors_s hitl_data;
	hitl_data.imu_ax = hitl_sensors.imu_ax;
	hitl_data.imu_ay = hitl_sensors.imu_ay;
	hitl_data.imu_az = hitl_sensors.imu_az;
	hitl_data.imu_gx = hitl_sensors.imu_gx;
	hitl_data.imu_gy = hitl_sensors.imu_gy;
	hitl_data.imu_gz = hitl_sensors.imu_gz;
	hitl_data.timestamp = _hal->get_time_us();

	_hitl_sensors_pub.publish(hitl_data);
}

void USBComm::transmit_hitl()
{
	aplink_hitl_commands hitl_commands;
	hitl_commands.ele_pwm = _hitl_output_data.ele_duty;
	hitl_commands.rud_pwm = _hitl_output_data.rud_duty;
	hitl_commands.thr_pwm = _hitl_output_data.thr_duty;

	uint8_t buffer[MAX_PACKET_LEN];
	uint16_t size = aplink_hitl_commands_pack(hitl_commands, buffer);
	_hal->usb_transmit(buffer, size);
}

void USBComm::transmit_vehicle_status()
{
	aplink_vehicle_status_full vehicle_status_full{};
	vehicle_status_full.roll = (int16_t)(_ahrs_data.roll * 100);
	vehicle_status_full.pitch = (int16_t)(_ahrs_data.pitch * 100);
	vehicle_status_full.yaw = (int16_t)(_ahrs_data.yaw * 100);
	vehicle_status_full.alt = 1;
	vehicle_status_full.spd = 2;
	vehicle_status_full.mode_id = get_mode_id();

	uint8_t packet[MAX_PACKET_LEN];
	uint16_t len = aplink_vehicle_status_full_pack(vehicle_status_full, packet);
	_hal->usb_transmit(packet, len);
}

uint8_t USBComm::get_mode_id()
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
