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
		if (validate_packet() && parse_packet())
		{
			ack();
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
	float sec_since_last_tlm_transmit = (_hal->get_time_us() - last_tlm_transmit_time) * us_to_s;

	uint16_t byte_rate = 0;
	if (sec_since_last_tlm_transmit != 0) // Prevent divide by zero
	{
		byte_rate = (bytes_since_last_tlm_transmit + TLM_PKT_LEN) / sec_since_last_tlm_transmit;
	}

	if (byte_rate < MAX_BYTE_RATE)
	{
		bytes_since_last_tlm_transmit = 0;
		last_tlm_transmit_time = _hal->get_time_us();

		// Construct telemetry packet
		Telem_payload payload = create_telem_payload();

		// Convert struct to byte array
		uint8_t payload_arr[sizeof(Telem_payload)];
		memcpy(payload_arr, &payload, sizeof(Telem_payload));

		// Encode with COBS
		uint8_t packet_cobs[sizeof(Telem_payload) + 1]; // Add 1 for COBS byte
		cobs_encode(packet_cobs, sizeof(packet_cobs), payload_arr, sizeof(Telem_payload));

		// Construct final packet
		uint8_t packet[TLM_PKT_LEN];
		packet[0] = 0; // Start byte
		packet[1] = sizeof(Telem_payload); // Length byte
		for (uint i = 0; i < sizeof(packet_cobs); i++)
		{
			packet[i + 2] = packet_cobs[i];
		}

		transmit_packet(packet, sizeof(packet));
	}
}

Telem_payload Telem::create_telem_payload()
{
	Telem_payload payload = {
		TELEM_MSG_ID,
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
		_plane->gps_fix,
		(int16_t)(-_plane->guidance_d_setpoint * 10)
	};

	return payload;
}

// Ensure packet length is valid before parsing
bool Telem::validate_packet()
{
	if (latest_pkt_len < 3) return false; // Prevent underflow in decoding
	if (latest_pkt_len > 255) return false; // Prevent buffer overflow
	return true;
}

// Send back same message for acknowledgement
void Telem::ack()
{
	// Do not use queue and send directly because this is priority
	transmit_packet(latest_packet, latest_pkt_len);
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
	if (msg_id == WPT_MSG_ID && payload_len == sizeof(Waypoint_payload))
	{
		Waypoint_payload waypoint_payload;
		memcpy(&waypoint_payload, payload, sizeof(Waypoint_payload));

		if (waypoint_payload.waypoint_index == 0)
		{
			_plane->home_lat = waypoint_payload.lat;
			_plane->home_lon = waypoint_payload.lon;
		}

		_plane->num_waypoints = waypoint_payload.total_waypoints;
		_plane->waypoints[waypoint_payload.waypoint_index] = (Waypoint){
			waypoint_payload.lat,
			waypoint_payload.lon,
			waypoint_payload.alt
		};

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
