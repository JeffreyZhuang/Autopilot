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

	if (params.set)
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
			_plane->mode_id,
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
	uint8_t payload[latest_pkt_len - 3]; // Subtract 3 since removed start, length, and COBS byte
	cobs_decode(payload, sizeof(payload), packet_cobs, sizeof(packet_cobs));

	uint8_t msg_id = payload[0];
	if (msg_id == CMD_MSG_ID)
	{
		Command_payload command_payload;
		memcpy(&command_payload, payload, sizeof(Command_payload));
		printf("%d\n", command_payload.command);

		return true;
	}
	else if (msg_id == WPT_MSG_ID)
	{
		Waypoint_payload waypoint_payload;
		memcpy(&waypoint_payload, payload, sizeof(Waypoint_payload));

		_plane->num_waypoints = waypoint_payload.waypoint_index + 1;
		_plane->waypoints[waypoint_payload.waypoint_index] = (Waypoint){waypoint_payload.lat, waypoint_payload.lon, waypoint_payload.alt};

		return true;
	}
	else if (msg_id == LND_TGT_MSG_ID)
	{
		Landing_target_payload landing_target_payload;
		memcpy(&landing_target_payload, payload, sizeof(Landing_target_payload));
		_plane->land_lat = landing_target_payload.lat;
		_plane->land_lon = landing_target_payload.lon;
		_plane->land_hdg = landing_target_payload.hdg;

		return true;
	}
	else if (msg_id == PARAMS_MSG_ID)
	{
		// Update if parameters haven't been set yet
		if (!params.set)
		{
			// Remove message ID
			uint8_t params_arr[sizeof(params)];
			for (uint i = 0; i < sizeof(params_arr); i++)
			{
				params_arr[i] = payload[i + 1];
			}

			// Copy parameters
			memcpy(&params, payload, sizeof(params));

			printf("Parameters set\n");

			return true;
		}
	}
	else
	{
		// Unrecognized command
	}

	return false;
}

bool Telem::compare_telem_payload(const struct Telem_payload *a, const struct Telem_payload *b) {
    return (a->payload_type == b->payload_type &&
            a->roll == b->roll &&
            a->pitch == b->pitch &&
            a->yaw == b->yaw &&
            a->alt == b->alt &&
            a->spd == b->spd &&
            a->lat == b->lat &&
            a->lon == b->lon &&
            a->mode_id == b->mode_id &&
            a->wp_idx == b->wp_idx &&
            a->gps_sats == b->gps_sats &&
            a->gps_fix == b->gps_fix);
}
