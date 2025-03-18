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
		_plane->of_x = result.x;
		_plane->of_y = result.y;
		_plane->of_timestamp = get_time_us();
	}
}
