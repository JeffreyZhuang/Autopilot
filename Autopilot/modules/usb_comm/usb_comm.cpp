#include "usb_comm.h"

USBComm::USBComm(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _pos_est_sub(data_bus->pos_est_node),
	  _ahrs_sub(data_bus->ahrs_node),
	  _gnss_sub(data_bus->gnss_node),
	  _modes_sub(data_bus->modes_node),
	  _position_control_sub(data_bus->position_control_node),
	  _waypoint_sub(data_bus->waypoint_node),
	  _power_sub(data_bus->power_node),
	  _ctrl_cmd_sub(data_bus->ctrl_cmd_node),
	  _hitl_output_sub(data_bus->hitl_output_node),
	  _baro_sub(data_bus->baro_node),
	  _telem_sub(data_bus->telem_node),
	  _home_pos_sub(data_bus->home_position_node),
	  _hitl_pub(data_bus->hitl_node)
{
}

void USBComm::update()
{
	// Read
	if (read_usb())
	{
		if (msg.msg_id == HITL_SENSORS_MSG_ID)
		{
			read_hitl();
		}
	}

	// Transmit
	if (param_get_int32(ENABLE_HITL))
	{
		transmit_hitl();
	}
	else
	{
		transmit_debug();
	}
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

	HITL_data hitl_data;
	hitl_data.imu_ax = hitl_sensors.imu_ax;
	hitl_data.imu_ay = hitl_sensors.imu_ay;
	hitl_data.imu_az = hitl_sensors.imu_az;
	hitl_data.imu_gx = hitl_sensors.imu_gx;
	hitl_data.imu_gy = hitl_sensors.imu_gy;
	hitl_data.imu_gz = hitl_sensors.imu_gz;

	_hitl_pub.publish(hitl_data);
}

void USBComm::transmit_hitl()
{
	HITL_output_data hitl_output_data = _hitl_output_sub.get();

	aplink_hitl_commands hitl_commands;
	hitl_commands.ele_pwm = hitl_output_data.ele_duty;
	hitl_commands.rud_pwm = hitl_output_data.rud_duty;
	hitl_commands.thr_pwm = hitl_output_data.thr_duty;

	uint8_t buffer[MAX_PACKET_LEN];
	uint16_t size = aplink_hitl_commands_pack(hitl_commands, buffer);

	_hal->usb_transmit(buffer, size);
}

void USBComm::transmit_debug()
{
	// Send comma separated values to web serial plotter for debug
	double gnss_north_meters, gnss_east_meters;
	lat_lon_to_meters(_home_pos_sub.get().lat, _home_pos_sub.get().lon,
					  _gnss_data.lat, _gnss_data.lon, &gnss_north_meters, &gnss_east_meters);

	char tx_buff[200];
	sprintf(tx_buff, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f\n",
			_pos_est_data.vel_n, _pos_est_data.vel_e, _pos_est_data.vel_d,
			_pos_est_data.pos_n, _pos_est_data.pos_e, _pos_est_data.pos_d,
			gnss_north_meters, gnss_east_meters, -(_baro_data.alt - _pos_est_data.baro_offset),
			_ahrs_data.yaw);

	_hal->usb_transmit((uint8_t*)tx_buff, strlen(tx_buff));
}
