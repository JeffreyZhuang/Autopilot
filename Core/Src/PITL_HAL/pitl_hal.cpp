#include "PITL_HAL/pitl_hal.h"

// ardupilot/libraries/SITL/SITL.h

Pitl_hal* Pitl_hal::_instance = nullptr;

Pitl_hal::Pitl_hal(Plane* plane)
	: mlrs_rc(&huart4),
	  mlrs_telem(&huart6)
{
	_plane = plane;
	_instance = this;
}

void Pitl_hal::init()
{
	mlrs_rc.setup();
	mlrs_telem.setup();
}

void Pitl_hal::read_sensors()
{
	read_pitl();
}
