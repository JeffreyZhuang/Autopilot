#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_of()
{
	cxof.setup();
}

void Flight_hal::read_of()
{
	Cxof_frame result;
	if (cxof.read(&result))
	{
		_plane->of_data.set(Plane::OF_data{result.x, result.y, get_time_us()});
	}
}
