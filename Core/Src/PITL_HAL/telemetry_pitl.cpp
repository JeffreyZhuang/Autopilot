#include "pitl_hal.h"
#include "Lib/Utils/utils.h"

void Pitl_hal::read_rc()
{
	uint16_t midpoint = (RC_IN_MAX + RC_IN_MIN) / 2;
	_plane->rc_rudder = map(mlrs_rc.rc_data[0], RC_IN_MIN, RC_IN_MAX, -1, 1);
	_plane->rc_elevator = map(mlrs_rc.rc_data[1], RC_IN_MIN, RC_IN_MAX, -1, 1);
	_plane->rc_throttle = map(mlrs_rc.rc_data[2], RC_IN_MIN, RC_IN_MAX, 0, 1);
	_plane->manual_sw = mlrs_rc.rc_data[4] > midpoint;
	_plane->mode_sw = mlrs_rc.rc_data[5] > midpoint;
}

void Pitl_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	mlrs_telem.transmit(tx_buff, len);
}

bool Pitl_hal::read_telem(uint8_t* rx_buff, int size)
{
	return mlrs_telem.read(rx_buff);
}
