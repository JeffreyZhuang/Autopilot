#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_telem()
{
	mlrs_rc.setup();
	mlrs_telem.setup();
}

void Flight_hal::read_rc()
{
	for (int i = 0; i < 6; i++)
	{
		_plane->rc_channels[i] = map(mlrs_rc.rc_data[i], params.rc_in_min, params.rc_in_max, -1, 1);
	}

	_plane->manual_sw = _plane->rc_channels[4] > 0.5;
	_plane->mode_sw = _plane->rc_channels[5] > 0.5;
}

void Flight_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	mlrs_telem.transmit(tx_buff, len);
}

bool Flight_hal::read_telem(uint8_t rx_buff[], uint8_t* size)
{
	return mlrs_telem.read(rx_buff, size);
}
