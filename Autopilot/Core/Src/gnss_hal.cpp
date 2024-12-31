#include <flight_hal.h>

void Derived_hal::init_gnss()
{
	_gnss.setup();
}

void Derived_hal::read_gnss()
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

void Derived_hal::gnss_dma_complete()
{
	_gnss.dma_complete();
}
