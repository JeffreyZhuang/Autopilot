#include "modules/telemetry/telem.h"

Telem::Telem(HAL* hal, Plane* plane) : Module(hal, plane)
{
}

void Telem::update()
{
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

	if (_plane->system_mode != System_mode::CONFIG)
	{
		transmit_telem();
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
			_plane->system_mode == System_mode::CONFIG &&
			_payload_len == sizeof(Waypoint_payload))
		{
			Waypoint_payload waypoint_payload;
			memcpy(&waypoint_payload, payload, sizeof(Waypoint_payload));

			if (waypoint_payload.waypoint_index == waypoint_payload.total_waypoints - 1)
			{
				_plane->waypoints_loaded = true;
			}

			_plane->num_waypoints = waypoint_payload.total_waypoints;
			_plane->waypoints[waypoint_payload.waypoint_index] = (Waypoint){
				(double)waypoint_payload.lat * 1E-7,
				(double)waypoint_payload.lon * 1E-7,
				(float)waypoint_payload.alt * 1E-1f
			};

			printf("Telem waypoint set\n");

			return true;
		}
		else if (_msg_id == PARAMS_MSG_ID &&
				 _plane->system_mode == System_mode::CONFIG &&
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
	AHRS_data ahrs_data = _plane->get_ahrs_data(ahrs_handle);
	GNSS_data gnss_data = _plane->get_gnss_data(gnss_handle);

	Telem_payload payload = {
		(int16_t)(ahrs_data.roll * 100),
		(int16_t)(ahrs_data.pitch * 100),
		(uint16_t)(ahrs_data.yaw * 10),
		(int16_t)(-_plane->nav_pos_down * 10),
		(uint16_t)(_plane->nav_gnd_spd * 10),
		(int16_t)(-_plane->guidance_d_setpoint * 10),
		(int32_t)(gnss_data.lat * 1E7),
		(int32_t)(gnss_data.lon * 1E7),
		_plane->nav_pos_north,
		_plane->nav_pos_east,
		get_current_state(),
		_plane->waypoint_index,
		0,
		0,
		0,
		(uint16_t)(_plane->autopilot_current * 1000.0f),
		gnss_data.sats,
		gnss_data.fix,
		(uint8_t)(_plane->rud_cmd * 100),
		(uint8_t)(_plane->ele_cmd * 100),
		(uint8_t)(_plane->thr_cmd * 100)
	};

	return payload;
}

// Returns unique state identifier
// Returns 255 if unknown state
uint8_t Telem::get_current_state()
{
    switch (_plane->system_mode)
    {
	case System_mode::CONFIG:
		return 0;
	case System_mode::STARTUP:
		return 1;
	case System_mode::FLIGHT:
		switch (_plane->flight_mode)
		{
		case Flight_mode::AUTO:
			switch (_plane->auto_mode)
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
			switch (_plane->manual_mode)
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
