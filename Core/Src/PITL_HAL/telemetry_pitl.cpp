#include "pitl_hal.h"

void Pitl_hal::read_rc()
{
	// Does not include throttle though which should be 0 to 1
	// Maybe use -1 to 1 instead?
	for (int i = 0; i < 6; i++)
	{
		_plane->rc_channels[i] = map(mlrs_rc.rc_data[i], params.rc_in_min, params.rc_in_max, -1, 1);
	}

	_plane->manual_sw = _plane->rc_channels[4] > 0.5;
	_plane->mode_sw = _plane->rc_channels[5] > 0.5;
}

void Pitl_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	mlrs_telem.transmit(tx_buff, len);
}

bool Pitl_hal::read_telem(uint8_t* rx_buff, uint8_t *size)
{
	return mlrs_telem.read(rx_buff, size);
}
