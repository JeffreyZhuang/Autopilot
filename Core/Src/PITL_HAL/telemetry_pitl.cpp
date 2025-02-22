#include "pitl_hal.h"

void Pitl_hal::read_rc()
{
	uint16_t midpoint = (params.rc_in_max + params.rc_in_min) / 2;
	_plane->rc_rudder = map(mlrs_rc.rc_data[0], params.rc_in_min, params.rc_in_max, -1, 1);
	_plane->rc_elevator = map(mlrs_rc.rc_data[1], params.rc_in_min, params.rc_in_max, -1, 1);
	_plane->rc_throttle = map(mlrs_rc.rc_data[2], params.rc_in_min, params.rc_in_max, 0, 1);
	_plane->manual_sw = mlrs_rc.rc_data[4] > midpoint;
	_plane->mode_sw = mlrs_rc.rc_data[5] > midpoint;
}

void Pitl_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	mlrs_telem.transmit(tx_buff, len);
}

bool Pitl_hal::read_telem(uint8_t* rx_buff, uint8_t *size)
{
	return mlrs_telem.read(rx_buff, size);
}
