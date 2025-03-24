#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_gnss()
{
	_gnss.setup();
}

void Flight_hal::read_gnss()
{
	if (_gnss.read())
	{
		_plane->set_gnss_data(GNSS_data{
			_gnss.lat,
			_gnss.lon,
			_gnss.alt,
			_gnss.sats,
			_gnss.fix,
			get_time_us()
		});
	}
}
