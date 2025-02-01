#include "telem.h"

#include <cstdio> // Temporary for printf debug

Telem::Telem(HAL* hal, Plane* plane)
{
	_plane = plane;
	_hal = hal;
}

void Telem::transmit()
{
	Telem_payload payload;
	payload.roll = _plane->ahrs_roll;
	payload.pitch = _plane->ahrs_pitch;
	payload.yaw = _plane->ahrs_yaw;
	payload.alt = -_plane->nav_pos_down;
	payload.spd = _plane->nav_airspeed;
	payload.lat = _plane->gnss_lat;
	payload.lon = _plane->gnss_lon;

	uint8_t payload_arr[sizeof(Telem_payload)];
	memcpy(payload_arr, &payload, sizeof(Telem_payload));

	uint8_t packet_no_start_byte[packet_len - 1]; // Packet without start byte, therefore subtract 1
	cobs_encode(packet_no_start_byte, sizeof(packet_no_start_byte), payload_arr, sizeof(Telem_payload));

	// Add start byte to packet
	uint8_t packet[packet_len];
	packet[0] = 0; // Start byte
	for (int i = 1; i < packet_len; i++)
	{
		packet[i] = packet_no_start_byte[i - 1];
	}

//	uint8_t test[] = {0};
//	_hal->transmit_telem(test, 1);
	_hal->transmit_telem(packet, packet_len);
}

void Telem::read()
{
	if (_hal->read_telem())
	{

	}
}
