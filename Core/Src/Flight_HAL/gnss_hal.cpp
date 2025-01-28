#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_gnss()
{
	_gnss.setup();
}

void Flight_hal::read_gnss()
{
	uint8_t sentence[100];
	if (_gnss.parse(sentence))
	{
		_plane->gnss_lat = _gnss.lat;
		_plane->gnss_lon = _gnss.lon;
		_plane->gnss_sats = _gnss.sats;
		_plane->gnss_timestamp = get_time_us();
	}
}
