#include "usb_comm.h"

USBComm::USBComm(HAL* hal, Data_bus* data_bus)
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
	  _hitl_output_sub(data_bus->hitl_output_node),
	  _baro_sub(data_bus->baro_node),
	  _telem_sub(data_bus->telem_node),
	  _hitl_pub(data_bus->hitl_node)
{
}

void USBComm::update()
{
	read();
	transmit();
}

void USBComm::read()
{
	while (!_hal->usb_buffer_empty())
	{
		uint8_t byte;
		_hal->usb_read(&byte);

		if (aplink_parse_byte(&msg, byte))
		{
			switch (msg.msg_id)
			{
			case HITL_INPUT_MSG_ID:
				aplink_hitl_input hitl_input;
				aplink_hitl_input_msg_decode(&msg, &hitl_input);

				HITL_data hitl_data;
				hitl_data.imu_ax = hitl_input.imu_ax;
				hitl_data.imu_ay = hitl_input.imu_ay;
				hitl_data.imu_az = hitl_input.imu_az;
				hitl_data.imu_gx = hitl_input.imu_gx;
				hitl_data.imu_gy = hitl_input.imu_gy;
				hitl_data.imu_gz = hitl_input.imu_gz;

				_hitl_pub.publish(hitl_data);

				break;
			}
		}
	}
}

void USBComm::transmit()
{
	if (_telem_data.hitl_enable)
	{
		HITL_output_data hitl_output_data = _hitl_output_sub.get();

		// Send to usb with autopilot link protocol
	}
	else
	{
		// Send CSV to web serial plotter

		double gnss_north_meters, gnss_east_meters;
		lat_lon_to_meters(_telem_data.waypoints[0].lat,
						  _telem_data.waypoints[0].lon,
						  _gnss_data.lat, _gnss_data.lon,
						  &gnss_north_meters, &gnss_east_meters);

		char tx_buff[200];
		sprintf(tx_buff,
				"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f\n",
				_pos_est_data.vel_n,
				_pos_est_data.vel_e,
				_pos_est_data.vel_d,
				_pos_est_data.pos_n,
				_pos_est_data.pos_e,
				_pos_est_data.pos_d,
				gnss_north_meters,
				gnss_east_meters,
				-(_baro_data.alt - _pos_est_data.baro_offset),
				_ahrs_data.yaw);

		_hal->usb_transmit((uint8_t*)tx_buff, strlen(tx_buff));
	}
}
