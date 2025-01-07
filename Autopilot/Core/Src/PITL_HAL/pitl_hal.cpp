#include "pitl_hal.h"

// ardupilot/libraries/SITL/SITL.h

Pitl_hal* Pitl_hal::_instance = nullptr;

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{
	_plane = plane;
	_instance = this;
}
