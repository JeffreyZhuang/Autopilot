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
	transmit();
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
	hitl_data.mag_x = hitl_sensors.mag_x;
	hitl_data.mag_y = hitl_sensors.mag_y;
	hitl_data.mag_z = hitl_sensors.mag_z;
	hitl_data.baro_asl = hitl_sensors.baro_asl;
	hitl_data.gps_lat = hitl_sensors.gps_lat;
	hitl_data.gps_lon = hitl_sensors.gps_lon;
	hitl_data.of_x = hitl_sensors.of_x;
	hitl_data.of_y = hitl_sensors.of_y;
	hitl_data.timestamp = _hal->get_time_us();

	_hitl_sensors_pub.publish(hitl_data);
}

void USBComm::transmit()
{
	aplink_hitl_commands hitl_commands;
	hitl_commands.ele_pwm = _hitl_output_data.ele_duty;
	hitl_commands.rud_pwm = _hitl_output_data.rud_duty;
	hitl_commands.thr_pwm = _hitl_output_data.thr_duty;

	aplink_vehicle_status_full vehicle_status_full{};
	vehicle_status_full.roll = (int16_t)(_ahrs_data.roll * 100);
	vehicle_status_full.pitch = (int16_t)(_ahrs_data.pitch * 100);
	vehicle_status_full.yaw = (int16_t)(_ahrs_data.yaw * 100);
	vehicle_status_full.alt = 1;
	vehicle_status_full.spd = 2;
	vehicle_status_full.mode_id = get_mode_id(
		_modes_data.system_mode,
		_modes_data.flight_mode,
		_modes_data.auto_mode,
		_modes_data.manual_mode
	);

	uint8_t buffer[MAX_PACKET_LEN * 2];
	uint16_t offset = 0;

	offset += aplink_hitl_commands_pack(hitl_commands, buffer + offset);
	offset += aplink_vehicle_status_full_pack(vehicle_status_full, buffer + offset);

	// You must transmit together in a single USB call!
	_hal->usb_transmit(buffer, offset);
}
