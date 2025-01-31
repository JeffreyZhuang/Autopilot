#include "telem.h"

#include <cstdio> // Temporary for printf debug

Telem::Telem(HAL* hal, Plane* plane)
{
	_plane = plane;
	_hal = hal;
}

void Telem::transmit()
{
	Telem_packet packet;
	packet.roll = _plane->ahrs_roll;
	packet.pitch = _plane->ahrs_pitch;
	packet.yaw = _plane->ahrs_yaw;
	packet.alt = -_plane->nav_pos_down;
	packet.spd = _plane->nav_airspeed;
	packet.lat = _plane->gnss_lat;
	packet.lon = _plane->gnss_lon;
	packet.sw = _plane->manual_sw;

	uint8_t tx_buff[sizeof(Telem_packet)];
	memcpy(tx_buff, &packet, sizeof(Telem_packet));
	_hal->transmit_telem(tx_buff, sizeof(tx_buff));
}
