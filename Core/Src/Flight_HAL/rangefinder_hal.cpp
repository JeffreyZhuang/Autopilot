#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_rangefinder()
{

}

void Flight_hal::read_rangefinder()
{
	_plane->rangefinder_dist = 0;
	_plane->rangefinder_timestamp = get_time_us();
}
