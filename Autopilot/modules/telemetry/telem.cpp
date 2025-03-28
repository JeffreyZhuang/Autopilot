#include "modules/telemetry/telem.h"

Telem::Telem(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _pos_est_sub(data_bus->pos_est_node),
	  _ahrs_sub(data_bus->ahrs_node),
	  _gnss_sub(data_bus->gnss_node),
	  _modes_sub(data_bus->modes_node),
	  _telem_sub(data_bus->telem_node),
	  _l1_sub(data_bus->l1_node),
	  _navigator_sub(data_bus->navigator_node),
	  _power_sub(data_bus->power_node),
	  _tecs_sub(data_bus->tecs_node),
	  _ctrl_cmd_sub(data_bus->ctrl_cmd_node),
	  _telem_pub(data_bus->telem_node)
{
}

void Telem::update()
{
	_modes_data = _modes_sub.get();

	while (!_hal->telem_buffer_empty())
	{
		uint8_t byte;
		_hal->read_telem(&byte);

		if (byte == START_BYTE)
		{
			_in_pkt = true;
			_pkt_idx = 0;
		}

		if (_in_pkt)
		{
			// Append byte to packet
			_packet[_pkt_idx] = byte;
			_pkt_idx++;

			switch (_pkt_idx)
			{
			case 1:
				break;
			case 2:
				_payload_len = byte;
				break;
			case 3:
				_msg_id = byte;
				break;
			case 4:
				_cobs_byte = byte;
				break;
			default:
				if (_pkt_idx == _payload_len + HEADER_LEN)
				{
					// Parse
					if (parse_packet())
					{
						ack();
					}

					// Reset
					_pkt_idx = 0;
					_in_pkt = false;
				}
				break;
			}
		}
	}

	if (_modes_data.system_mode != System_mode::CONFIG)
	{
		_ahrs_data = _ahrs_sub.get();
		_gnss_data = _gnss_sub.get();
		_pos_est_data = _pos_est_sub.get();

		transmit_telem();
	}

	_telem_pub.publish(_telem_data);
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

		// Construct telemetry payload
		Telem_payload payload = create_telem_payload();

		// Convert struct to byte array
		uint8_t payload_arr[sizeof(Telem_payload)];
		memcpy(payload_arr, &payload, sizeof(Telem_payload));

		// Encode with COBS
		uint8_t packet_cobs[sizeof(Telem_payload) + 1]; // Add 1 for COBS byte
		cobs_encode(packet_cobs, sizeof(packet_cobs), payload_arr, sizeof(Telem_payload));

		// Construct final packet
		uint16_t packet_index = 0;
		uint8_t packet[sizeof(Telem_payload) + HEADER_LEN];
		packet[packet_index++] = START_BYTE; // Start byte
		packet[packet_index++] = sizeof(Telem_payload); // Length byte
		packet[packet_index++] = TELEM_MSG_ID; // Message ID
		for (uint i = 0; i < sizeof(packet_cobs); i++)
		{
			packet[packet_index++] = packet_cobs[i];
		}

		transmit_packet(packet, sizeof(packet));
	}
}

bool Telem::parse_packet()
{
	printf("Telem msg id: %d\n", _msg_id);
	printf("Telem payload len: %d\n", _payload_len);
	printf("Telem waypoint_payload len: %d\n", sizeof(Waypoint_payload));
	printf("Telem params_payload len: %d\n", sizeof(Params_payload));

	// Remove header except COBS byte
	uint8_t payload_cobs[_payload_len + 1]; // Add 1 for COBS byte
	payload_cobs[0] = _cobs_byte;
	for (uint i = 0; i < _payload_len; i++)
	{
		payload_cobs[i + 1] = _packet[i + HEADER_LEN];
	}

	// Decode consistent overhead byte shuffling
	uint8_t payload[_payload_len];
	cobs_decode_result res = cobs_decode(payload, sizeof(payload), payload_cobs, sizeof(payload_cobs));
	if (res.status == COBS_DECODE_OK)
	{
		// Determine type of payload from message ID
		if (_msg_id == WPT_MSG_ID &&
			_modes_data.system_mode == System_mode::CONFIG &&
			_payload_len == sizeof(Waypoint_payload))
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

			return true;
		}
		else if (_msg_id == PARAMS_MSG_ID &&
				 _modes_data.system_mode == System_mode::CONFIG &&
				 _payload_len == sizeof(Params_payload))
		{
			Params_payload params_payload;
			memcpy(&params_payload, payload, sizeof(Params_payload));

			// Set parameters
			set_params(&params_payload.params);

			printf("Telem params set\n");

			return true;
		}
	}
	else
	{
		printf("COBS DECODE NOT OK\n");
	}

	return false;
}

// Send back same message for acknowledgement
void Telem::ack()
{
	// Do not use queue and send directly because this is priority
	transmit_packet(_packet, _payload_len + HEADER_LEN);
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
			case Auto_mode::TOUCHDOWN:
				return 6;
			}
		case Flight_mode::MANUAL:
			switch (_modes_data.manual_mode)
			{
				case Manual_mode::DIRECT:
					return 7;
				case Manual_mode::STABILIZED:
					return 8;
			}
		}
    }

    return 255;
}
