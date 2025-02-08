#include "telem.h"

#include <cstdio> // Temporary for printf debug

Telem::Telem(HAL* hal, Plane* plane)
{
	_plane = plane;
	_hal = hal;

	prev_transmit_time = _hal->get_time_us();
}

void Telem::transmit()
{
	// Create struct
	Telem_payload payload;
	payload.roll = _plane->ahrs_roll;
	payload.pitch = _plane->ahrs_pitch;
	payload.yaw = _plane->ahrs_yaw;
	payload.alt = -_plane->nav_pos_down;
	payload.spd = _plane->nav_airspeed;
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

	_hal->transmit_telem(packet, TELEM_PKT_LEN);
}

// Send back same message
void Telem::acknowledgement()
{
	_hal->transmit_telem(_plane->latest_packet, TELEM_PKT_LEN);
}

void Telem::parse_telemetry()
{
	// Remove start byte
	uint8_t packet_no_start_byte[TELEM_PKT_LEN - 1];
	for (int i = 0; i < TELEM_PKT_LEN - 1; i++)
	{
		packet_no_start_byte[i] = _plane->latest_packet[i + 1];
	}

	// Decode consistent overhead byte shuffling
	uint8_t payload[TELEM_PKT_LEN - 2]; // Subtract two since removed start byte and COBS byte
	cobs_decode(payload, sizeof(payload), packet_no_start_byte, sizeof(packet_no_start_byte)); // Need to handle the case where 0 is not found?

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
}

void Telem::update()
{
	// If new recieved command, send acknowledgement
	// Otherwise send telemetry packet
	if (_hal->read_telem())
	{
		parse_telemetry();
		acknowledgement();
		prev_transmit_time = _hal->get_time_us();
	}
	else
	{
		// Limit to 10Hz to prevent overflowing buffer
		if (_hal->get_time_us() - prev_transmit_time > 100000)
		{
			transmit();
			prev_transmit_time = _hal->get_time_us();
		}
	}
}
