#include "telem.h"

Telem::Telem(HAL* hal, Plane* plane)
{
	_plane = plane;
	_hal = hal;
}

void Telem::update()
{
	if (_hal->read_telem(latest_packet, &latest_pkt_len))
	{
		if (parse_packet())
		{
			ack();
		}
	}

	if (_plane->system_mode != System_mode::CONFIG)
	{
		transmit_telem();
	}
}

void Telem::transmit(uint8_t packet[], uint16_t size)
{
	_hal->transmit_telem(packet, size);
	total_bytes_sent += size;

	// Track time of first transmit
	if (start_time == 0)
	{
		start_time = _hal->get_time_us();
	}
}

// The timer approach fails when disconnecting GCS and then connecting again
void Telem::transmit_telem()
{
	// Limit serial rate through radio
	float dt = (_hal->get_time_us() - start_time) * us_to_s;
	uint16_t serial_rate = (total_bytes_sent + telem_packet_len) / dt;
	if (serial_rate < max_serial_rate)
	{
		// Create struct
		Telem_payload payload = {
			0,
			(int16_t)(_plane->ahrs_roll * 100),
			(int16_t)(_plane->ahrs_pitch * 100),
			(uint16_t)(_plane->ahrs_yaw * 10),
			(int16_t)(-_plane->nav_pos_down * 10),
			(uint16_t)(_plane->nav_airspeed * 10),
			(float)_plane->gnss_lat,
			(float)_plane->gnss_lon,
			get_current_state(),
			_plane->waypoint_index,
			_plane->gnss_sats,
			_plane->gps_fix
		};

		// Convert struct to byte array
		uint8_t payload_arr[sizeof(Telem_payload)];
		memcpy(payload_arr, &payload, sizeof(Telem_payload));

		// Consistent overhead byte stuffing
		uint8_t packet_cobs[sizeof(Telem_payload) + 1]; // Add 1 for COBS byte
		cobs_encode(packet_cobs, sizeof(packet_cobs), payload_arr, sizeof(Telem_payload));

		// Add start and length bytes to beginning of packet
		uint8_t packet[telem_packet_len]; // Add 3 for start byte, length byte, and COBS byte
		packet[0] = 0; // Start byte
		packet[1] = sizeof(Telem_payload); // Length byte
		for (uint i = 0; i < sizeof(packet_cobs); i++)
		{
			packet[i + 2] = packet_cobs[i];
		}

		total_bytes_sent = 0;
		start_time = _hal->get_time_us();

		transmit(packet, sizeof(packet));
	}
}

// Send back same message for acknowledgement
void Telem::ack()
{
	// Do not use queue and send directly because this is priority
	transmit(latest_packet, latest_pkt_len);
}

bool Telem::parse_packet()
{
	// Remove start and length byte
	uint8_t packet_cobs[latest_pkt_len - 2];
	for (uint i = 0; i < sizeof(packet_cobs); i++)
	{
		packet_cobs[i] = latest_packet[i + 2];
	}

	// Decode consistent overhead byte shuffling
	uint8_t payload[latest_pkt_len - 3]; // Subtract 3 since removed header
	cobs_decode(payload, sizeof(payload), packet_cobs, sizeof(packet_cobs));

	uint16_t payload_len = latest_pkt_len - 3; // Subtract header
	uint8_t msg_id = payload[0];
	printf("Telem msg_id: %d\n", msg_id);
	printf("Telem payload_len: %d\n", payload_len);
	printf("Telem params len: %d\n", sizeof(Parameters));
	if (msg_id == CMD_MSG_ID && payload_len == sizeof(Command_payload))
	{
		Command_payload command_payload;
		memcpy(&command_payload, payload, sizeof(Command_payload));
		printf("Command: %d\n", command_payload.command);

		return true;
	}
	else if (msg_id == WPT_MSG_ID && payload_len == sizeof(Waypoint_payload))
	{
		Waypoint_payload waypoint_payload;
		memcpy(&waypoint_payload, payload, sizeof(Waypoint_payload));

		_plane->num_waypoints = waypoint_payload.waypoint_index + 1;
		_plane->waypoints[waypoint_payload.waypoint_index] = (Waypoint){
			waypoint_payload.waypoint_type,
			waypoint_payload.lat,
			waypoint_payload.lon,
			waypoint_payload.alt
		};

		return true;
	}
	else if (msg_id == PARAMS_MSG_ID &&
			 _plane->system_mode == System_mode::CONFIG &&
			 payload_len - 1 == sizeof(Parameters))
	{
		// Remove message ID
		uint8_t params_arr[sizeof(Parameters)];
		for (uint i = 0; i < sizeof(params_arr); i++)
		{
			params_arr[i] = payload[i + 1];
		}

		// Create Parameters struct from bytes
		Parameters temp_params;
		memcpy(&temp_params, params_arr, sizeof(Parameters));

		// Set parameters
		set_params(&temp_params);

		printf("Parmeters set");

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
			default:
				return 255;
			}
		case Flight_mode::MANUAL:
			switch (_plane->manual_mode)
			{
				case Manual_mode::DIRECT:
					return 7;
				case Manual_mode::STABILIZED:
					return 8;
				default:
					return 255;
			}
		default:
			return 255;
		}
	default:
		return 255;
    }
}
