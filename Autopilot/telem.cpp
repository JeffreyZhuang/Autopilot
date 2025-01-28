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
	packet.sw = _plane->rc_switch;

	uint8_t tx_buff[sizeof(Telem_packet)];
	memcpy(tx_buff, &packet, sizeof(Telem_packet));
	_hal->transmit_telem(tx_buff, sizeof(tx_buff));
}
