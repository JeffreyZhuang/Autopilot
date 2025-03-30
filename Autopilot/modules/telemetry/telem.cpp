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
	  _hitl_output_sub(data_bus->hitl_output_node),
	  _baro_sub(data_bus->baro_node),
	  _telem_pub(data_bus->telem_node),
	  _hitl_pub(data_bus->hitl_node)
{
}

void Telem::update()
{
	_modes_data = _modes_sub.get();

	read_telem();
	read_usb();

	_telem_pub.publish(_telem_data);

	if (_modes_data.system_mode != System_mode::CONFIG)
	{
		_ahrs_data = _ahrs_sub.get();
		_gnss_data = _gnss_sub.get();
		_pos_est_data = _pos_est_sub.get();

		transmit_telem();
		transmit_usb();
	}
}

void Telem::read_telem()
{
	while (!_hal->telem_buffer_empty())
	{
		uint8_t byte;
		_hal->read_telem(&byte);

		uint8_t payload[MAX_PAYLOAD_LEN];
		uint8_t payload_len;
		uint8_t msg_id;
		if (telem_link.parse_byte(byte, payload, payload_len, msg_id))
		{
			if (msg_id == WPT_MSG_ID &&
				_modes_data.system_mode == System_mode::CONFIG &&
				payload_len == sizeof(Waypoint_payload))
			{
				Waypoint_payload waypoint_payload;
				memcpy(&waypoint_payload, payload, sizeof(Waypoint_payload));

				if (waypoint_payload.waypoint_index == waypoint_payload.total_waypoints - 1)
				{
					_telem_data.waypoints_loaded = true;
				}

				_telem_data.num_waypoints = waypoint_payload.total_waypoints;
				_telem_data.waypoints[waypoint_payload.waypoint_index] = Waypoint{
					(double)waypoint_payload.lat * 1E-7,
					(double)waypoint_payload.lon * 1E-7,
					(float)waypoint_payload.alt * 1E-1f
				};

				printf("Telem waypoint set\n");
			}
			else if (msg_id == PARAMS_MSG_ID &&
					 _modes_data.system_mode == System_mode::CONFIG &&
					 payload_len == sizeof(Params_payload))
			{
				Params_payload params_payload;
				memcpy(&params_payload, payload, sizeof(Params_payload));

				// Set parameters
				_telem_data.params = params_payload;
				_telem_data.params_loaded = true;

				printf("Telem params set\n");
			}

			ack();
		}
	}
}

void Telem::read_usb()
{
	while (!_hal->usb_buffer_empty())
	{
		uint8_t byte;
		_hal->usb_read(&byte);

		uint8_t payload[MAX_PAYLOAD_LEN];
		uint8_t payload_len;
		uint8_t msg_id;
		if (usb_link.parse_byte(byte, payload, payload_len, msg_id))
		{
			if (msg_id == HITL_MSG_ID)
			{
//				_hitl_pub.publish(HITL_data{});
			}
		}
	}
}

void Telem::transmit_usb()
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

void Telem::transmit_telem()
{
	// Limit data rate through radio
	float sec_since_last_tlm_transmit = (_hal->get_time_us() - _last_tlm_transmit_time) * US_TO_S;

	uint16_t byte_rate = 0;
	if (sec_since_last_tlm_transmit != 0) // Prevent divide by zero
	{
		byte_rate = (_bytes_since_last_tlm_transmit + sizeof(Telem_payload) + HEADER_LEN) / sec_since_last_tlm_transmit;
	}

	if (byte_rate < MAX_BYTE_RATE)
	{
		_bytes_since_last_tlm_transmit = 0;
		_last_tlm_transmit_time = _hal->get_time_us();

		Telem_payload payload = create_telem_payload();

		uint8_t packet[telem_link.calc_packet_size(sizeof(payload))];
		telem_link.pack(packet, reinterpret_cast<uint8_t*>(&payload), sizeof(Telem_payload), TELEM_MSG_ID);

		transmit_packet(packet, sizeof(packet));
	}
}

// Send back same message for acknowledgement
void Telem::ack()
{
	// Do not use queue and send directly because this is priority
	transmit_packet(telem_link.latest_packet, telem_link.latest_packet_len);
}

void Telem::transmit_packet(uint8_t packet[], uint16_t size)
{
	_bytes_since_last_tlm_transmit += size;
	_hal->transmit_telem(packet, size);
}

Telem_payload Telem::create_telem_payload()
{
	Telem_payload payload = {
		(int16_t)(_ahrs_data.roll * 100),
		(int16_t)(_ahrs_data.pitch * 100),
		(uint16_t)(_ahrs_data.yaw * 10),
		(int16_t)(-_pos_est_data.pos_d * 10),
		(uint16_t)(_pos_est_data.gnd_spd * 10),
		(int16_t)(-_l1_data.d_setpoint * 10),
		(int32_t)(_gnss_data.lat * 1E7),
		(int32_t)(_gnss_data.lon * 1E7),
		_pos_est_data.pos_n,
		_pos_est_data.pos_e,
		get_current_state(),
		_navigator_data.waypoint_index,
		0,
		0,
		0,
		(uint16_t)(_power_data.autopilot_current * 1000.0f),
		_gnss_data.sats,
		_gnss_data.fix,
		(uint8_t)(_ctrl_cmd_data.rud_cmd * 100),
		(uint8_t)(_ctrl_cmd_data.ele_cmd * 100),
		(uint8_t)(_tecs_data.thr_cmd * 100)
	};

	return payload;
}

// Returns unique state identifier
// Returns 255 if unknown state
uint8_t Telem::get_current_state()
{
    switch (_modes_data.system_mode)
    {
	case System_mode::CONFIG:
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
