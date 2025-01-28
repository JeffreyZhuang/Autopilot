#include "pitl_hal.h"

void Pitl_hal::read_rc()
{
	// Need to add some mixer
	// Convert from 1000 to 2000 to -1 to 1 range
	_plane->rc_rudder = ((int)mlrs.rc_data[0] - 1500) / 500.0f;
	_plane->rc_elevator = -((int)mlrs.rc_data[1] - 1500) / 500.0f;

	// 0 to 1 range
	_plane->rc_throttle = ((int)mlrs.rc_data[2] - 1000) / 1000.0f;

	_plane->rc_switch = mlrs.rc_data[5] > 1500;
}

void Pitl_hal::transmit_telem()
{

}
