#include "telem.h"

Telem::Telem(HAL* hal, Plane* plane)
{
	_plane = plane;
	_hal = hal;
}

void Telem::update()
{
	while (!_hal->telem_buffer_empty())
	{
		uint8_t byte;
		_hal->read_telem(&byte);

		if (byte == START_BYTE)
		{
			in_packet = true;
		}

		if (in_packet)
		{
			// Append byte to packet
			packet[packet_index] = byte;
			packet_index++;

			if (packet_index == 1)
			{
				// Ignore start byte
			}
			else if (packet_index == 2)
			{
				payload_len = byte;
			}
			else if (packet_index == 3)
			{
				msg_id = byte;
			}
			else if (packet_index == 4)
			{
				cobs_byte = byte;
			}
			else if (packet_index == payload_len)
			{
				// Parse
				if (parse_packet())
				{
					ack();
				}

				// Reset
				packet_index = 0;
				in_packet = false;
			}
		}
	}

	if (_plane->system_mode != System_mode::CONFIG)
	{
		transmit_telem();
	}
}

void Telem::transmit_packet(uint8_t packet[], uint16_t size)
{
	bytes_since_last_tlm_transmit += size;
	_hal->transmit_telem(packet, size);
}

void Telem::transmit_telem()
{
	// Limit data rate through radio
	float sec_since_last_tlm_transmit = (_hal->get_time_us() - last_tlm_transmit_time) * US_TO_S;

	uint16_t byte_rate = 0;
	if (sec_since_last_tlm_transmit != 0) // Prevent divide by zero
	{
		byte_rate = bytes_since_last_tlm_transmit / sec_since_last_tlm_transmit;
	}

	if (byte_rate < MAX_BYTE_RATE)
	{
		bytes_since_last_tlm_transmit = 0;
		last_tlm_transmit_time = _hal->get_time_us();

		// Construct telemetry payload
		Telem_payload payload = create_telem_payload();

		// Convert struct to byte array
		uint8_t payload_arr[sizeof(Telem_payload)];
		memcpy(payload_arr, &payload, sizeof(Telem_payload));

		// Encode with COBS
		uint8_t packet_cobs[sizeof(Telem_payload) + 1]; // Add 1 for COBS byte
		cobs_encode(packet_cobs, sizeof(packet_cobs), payload_arr, sizeof(Telem_payload));

		uint16_t packet_index = 0;

		// Construct final packet
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

Telem_payload Telem::create_telem_payload()
{
	Telem_payload payload = {
		(int16_t)(_plane->ahrs_roll * 100),
		(int16_t)(_plane->ahrs_pitch * 100),
		(uint16_t)(_plane->ahrs_yaw * 10),
		(int16_t)(-_plane->nav_pos_down * 10),
		(uint16_t)(_plane->nav_gnd_spd * 10),
		(int16_t)(-_plane->guidance_d_setpoint * 10),
		(int32_t)(_plane->gnss_lat * 1E7),
		(int32_t)(_plane->gnss_lon * 1E7),
		_plane->nav_pos_north,
		_plane->nav_pos_east,
		get_current_state(),
		_plane->waypoint_index,
		0,
		0,
		0,
		0,
		_plane->gnss_sats,
		_plane->gps_fix,
		(uint8_t)(_plane->rud_cmd * 100),
		(uint8_t)(_plane->ele_cmd * 100),
		(uint8_t)(_plane->thr_cmd * 100)
	};

	return payload;
}

// Send back same message for acknowledgement
void Telem::ack()
{
	// Do not use queue and send directly because this is priority
	transmit_packet(packet, payload_len + HEADER_LEN);
}

bool Telem::parse_packet()
{
	printf("Telem msg id: %d\n", msg_id);
	printf("Telem payload len: %d\n", payload_len);
	printf("Telem waypoint_payload len: %d\n", sizeof(Waypoint_payload));
	printf("Telem params_payload len: %d\n", sizeof(Params_payload));

	// Remove header except COBS byte
	uint8_t payload_cobs[payload_len + 1]; // Add 1 for COBS byte
	payload_cobs[0] = cobs_byte;
	for (uint i = 1; i < sizeof(payload_cobs); i++)
	{
		payload_cobs[i] = packet[i + HEADER_LEN - 1];
	}

	// Decode consistent overhead byte shuffling
	uint8_t payload[payload_len];
	cobs_decode(payload, sizeof(payload), payload_cobs, sizeof(payload_cobs));

	// Determine type of payload from message ID
	if (msg_id == WPT_MSG_ID &&
		_plane->system_mode == System_mode::CONFIG &&
		payload_len == sizeof(Waypoint_payload))
	{
		Waypoint_payload waypoint_payload;
		memcpy(&waypoint_payload, payload, sizeof(Waypoint_payload));

		if (waypoint_payload.waypoint_index == waypoint_payload.total_waypoints - 1)
		{
			_plane->waypoints_loaded = true;
		}

		if (waypoint_payload.waypoint_index == 0)
		{
			_plane->home_lat = (float)waypoint_payload.lat * 1E-7f;
			_plane->home_lon = (float)waypoint_payload.lon * 1E-7f;
		}

		_plane->num_waypoints = waypoint_payload.total_waypoints;
		_plane->waypoints[waypoint_payload.waypoint_index] = (Waypoint){
			(float)waypoint_payload.lat * 1E-7f,
			(float)waypoint_payload.lon * 1E-7f,
			(float)waypoint_payload.alt * 1E-1f
		};

		printf("Telem waypoint set\n");

		return true;
	}
	else if (msg_id == PARAMS_MSG_ID &&
			 _plane->system_mode == System_mode::CONFIG &&
			 payload_len == sizeof(Params_payload))
	{
		Params_payload params_payload;
		memcpy(&params_payload, payload, sizeof(Params_payload));

		// Set parameters
		set_params(&params_payload.params);

		printf("Telem params set\n");

		return true;
	}
	return false;
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
