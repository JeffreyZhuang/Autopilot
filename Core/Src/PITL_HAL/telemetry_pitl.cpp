#include "pitl_hal.h"
#include "utils.h"

void Pitl_hal::read_rc()
{
	// Need to add some mixer
	// Convert from 1000 to 2000 to -1 to 1 range
	_plane->rc_rudder = clamp(((int)mlrs_rc.rc_data[0] - 1500) / 500.0f, -1, 1);
	_plane->rc_elevator = clamp(-((int)mlrs_rc.rc_data[1] - 1500) / 500.0f, -1, 1);

	// 0 to 1 range
	_plane->rc_throttle = clamp(((int)mlrs_rc.rc_data[2] - 1000) / 1000.0f, 0, 1);

	_plane->manual_sw = mlrs_rc.rc_data[5] > 1500;
	_plane->mode_sw = mlrs_rc.rc_data[6] > 1500;
}

void Pitl_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	mlrs_telem.transmit(tx_buff, len);
}

bool Pitl_hal::read_telem()
{
	uint8_t packet[TELEM_PKT_LEN];

	if (mlrs_telem.read(packet))
	{
		memcpy(_plane->latest_packet, packet, sizeof(packet));
		return true;
	}

	return false;
}
