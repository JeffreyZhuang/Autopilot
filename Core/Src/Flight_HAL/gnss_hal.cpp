#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_gnss()
{
	_gnss.setup();
}

bool Flight_hal::read_gnss(double *lat, double *lon, float* alt, uint8_t* sats, bool* fix)
{
	if (_gnss.read())
	{
		*lat = _gnss.lat;
		*lon = _gnss.lon;
		*alt = _gnss.alt;
		*sats = _gnss.sats;
		*fix = _gnss.fix;

		return true;
	}

	return false;
}
