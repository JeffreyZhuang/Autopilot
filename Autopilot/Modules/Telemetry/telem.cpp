#include "telem.h"

// Need to add queue! Acknowledgement packet might get rejected and will never execute again

Telem::Telem(HAL* hal, Plane* plane)
{
	_plane = plane;
	_hal = hal;

	start_time = _hal->get_time_us();
}

void Telem::update()
{
	// If recieved command, send acknowledgement
	// Otherwise send telemetry packet
	uint8_t read_size;
	if (_hal->read_telem(latest_packet, &read_size))
	{
		parse_telemetry();
		acknowledgement();
	}
	else
	{
		transmit();
	}
}

void Telem::transmit()
{
	// transmit nav_pos_north and nav_pos_east meters*100

	// Create struct
	Telem_payload payload;
	payload.roll = (int16_t)(_plane->ahrs_roll * 100);
	payload.pitch = (int16_t)(_plane->ahrs_pitch * 100);
	payload.yaw = (uint16_t)(_plane->ahrs_yaw * 10);
	payload.alt = (int16_t)(-_plane->nav_pos_down * 10);
	payload.spd = (uint16_t)(_plane->nav_airspeed * 10);
	payload.lat = _plane->gnss_lat;
	payload.lon = _plane->gnss_lon;
	payload.mode_id = _plane->mode_id;
	payload.wp_idx = _plane->waypoint_index;
	payload.gps_sats = _plane->gnss_sats;
	payload.gps_fix = _plane->gps_fix;

	// Convert struct to byte array
	uint8_t payload_arr[sizeof(Telem_payload)];
	memcpy(payload_arr, &payload, sizeof(Telem_payload));

	// Consistent overhead byte stuffing
	uint8_t packet_no_start_byte[TELEM_PKT_LEN - 1]; // Packet without start byte, therefore subtract 1
	cobs_encode(packet_no_start_byte, sizeof(packet_no_start_byte), payload_arr, sizeof(Telem_payload));

	// Add start byte to beginning of packet
	uint8_t packet[TELEM_PKT_LEN];
	packet[0] = 0; // Start byte
	for (int i = 1; i < TELEM_PKT_LEN; i++)
	{
		packet[i] = packet_no_start_byte[i - 1];
	}

	send(packet, TELEM_PKT_LEN);
}

// Send back same message
void Telem::acknowledgement()
{
	send(latest_packet, TELEM_PKT_LEN);
}

void Telem::parse_telemetry()
{
	// Remove start byte
	uint8_t packet_no_start_byte[TELEM_PKT_LEN - 1];
	for (int i = 0; i < TELEM_PKT_LEN - 1; i++)
	{
		packet_no_start_byte[i] = latest_packet[i + 1];
	}

	// Decode consistent overhead byte shuffling
	uint8_t payload[TELEM_PKT_LEN - 2]; // Subtract two since removed start byte and COBS byte
	cobs_decode(payload, sizeof(payload), packet_no_start_byte, sizeof(packet_no_start_byte));

	if (payload[0] == 1) // Command payload
	{
		Command_payload command_payload;
		memcpy(&command_payload, payload, sizeof(Command_payload));
//		printf("%d\n", command_payload.command);
	}
	else if (payload[0] == 2) // Waypoint payload
	{
		Waypoint_payload waypoint_payload;
		memcpy(&waypoint_payload, payload, sizeof(Waypoint_payload));

		_plane->num_waypoints = waypoint_payload.waypoint_index + 1; // Add a byte to indicate max number of waypoints later
		_plane->waypoints[waypoint_payload.waypoint_index] = (Waypoint){waypoint_payload.lat, waypoint_payload.lon, waypoint_payload.alt};
	}
	else if (payload[0] == 3) // Landing target payload
	{
		Landing_target_payload landing_target_payload;
		memcpy(&landing_target_payload, payload, sizeof(Landing_target_payload));
		_plane->land_lat = landing_target_payload.lat;
		_plane->land_lon = landing_target_payload.lon;
		_plane->land_hdg = landing_target_payload.hdg;
	}
	else if (payload[0] == 4) // Parameters payload
	{
		// Load parameters
//		static Parameters params;
//		memcpy(&params, payload, sizeof(params));
//		Params = &params;
	}
	else
	{
		// Unrecognized command
	}
}

void Telem::send(uint8_t* packet, uint8_t size)
{
	uint64_t dt = _hal->get_time_us() - start_time;
	uint16_t serial_rate = (total_bytes_sent + size) / dt;

	if (serial_rate < max_serial_rate)
	{
		_hal->transmit_telem(packet, size);
		total_bytes_sent += size;
	}
}

void Telem::append_queue(uint8_t* packet, uint8_t size)
{
	// Check if message is already in queue
	bool in_queue = false;
	for (int i = 0; i < current_queue_idx; i++)
	{
		if (arrays_are_equal(queue[i], packet, size))
		{
			in_queue = true;
			break;
		}
	}

	// Add to queue if its not already present
	// And if queue is not full
	bool queue_full = current_queue_idx == queue_len;
	if (!in_queue && !queue_full)
	{
		memcpy(queue[current_queue_idx], packet, size);
		current_queue_idx++;
	}
}

bool Telem::arrays_are_equal(uint8_t arr1[], uint8_t arr2[], uint8_t size) {
    for (int i = 0; i < size; i++) {
        if (arr1[i] != arr2[i]) {
            return false;
        }
    }
    return true;
}
