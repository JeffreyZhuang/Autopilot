#include "pitl_hal.h"

// ardupilot/libraries/SITL/SITL.h

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{
	_plane = plane;
}
