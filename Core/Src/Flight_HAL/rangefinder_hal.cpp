#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_rangefinder()
{
	cxof.setup();
}

void Flight_hal::read_rangefinder()
{
	Cxof_frame result;
	if (cxof.read(&result))
	{
		_plane->of_x = result.x;
		_plane->of_y = result.y;
		_plane->of_timestamp = get_time_us();
	}
}
