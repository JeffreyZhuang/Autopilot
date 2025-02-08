#include "Flight_HAL/flight_hal.h"
#include "Lib/Utils/utils.h"

void Flight_hal::init_telem()
{
	mlrs_rc.setup();
	mlrs_telem.setup();
}

void Flight_hal::read_rc()
{
	// Convert range from (1000, 2000) to (-1, 1)
	_plane->rc_rudder = clamp(((int)mlrs_rc.rc_data[0] - 1500) / 500.0f, -1, 1);
	_plane->rc_elevator = clamp(-((int)mlrs_rc.rc_data[1] - 1500) / 500.0f, -1, 1);

	// (0, 1) range
	_plane->rc_throttle = clamp(((int)mlrs_rc.rc_data[2] - 1000) / 1000.0f, 0, 1);

	_plane->manual_sw = mlrs_rc.rc_data[4] > 1500;
	_plane->mode_sw = mlrs_rc.rc_data[5] > 1500;
}

void Flight_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	mlrs_telem.transmit(tx_buff, len);
}

bool Flight_hal::read_telem()
{
	return mlrs_telem.read(_plane->latest_packet);
}
