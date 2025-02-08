#include "pitl_hal.h"

// ardupilot/libraries/SITL/SITL.h

Pitl_hal* Pitl_hal::_instance = nullptr;

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane), mlrs_rc(&huart4), mlrs_telem(&huart6)
{
	_plane = plane;
	_instance = this;

	main_dt = 0.02;
	control_dt = 0.04;
}

void Pitl_hal::init()
{
	mlrs_rc.setup();
	mlrs_telem.setup();
}

void Pitl_hal::read_sensors()
{
	read_rc();
	read_pitl();
}
