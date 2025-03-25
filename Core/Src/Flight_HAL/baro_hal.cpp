#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_baro()
{
	Barometer_init();
	Barometer_setOSR(OSR_4096);
}

void Flight_hal::read_baro()
{
	float alt;
	if (Barometer_getAltitude(&alt))
	{
		_plane->baro_data.set(Plane::Baro_data{alt, get_time_us()});
	}
}
