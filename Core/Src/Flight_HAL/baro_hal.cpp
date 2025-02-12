#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_baro()
{
	Barometer_init();
	Barometer_setOSR(OSR_4096);
}

void Flight_hal::read_baro()
{
	if (Barometer_getAltitude(&_plane->baro_alt))
	{
		_plane->baro_timestamp = get_time_us();
	}
}
