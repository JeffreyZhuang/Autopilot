#include <flight_hal.h>

void Flight_hal::init_baro()
{
	Barometer_init();
	Barometer_setOSR(OSR_4096);
}

void Flight_hal::read_baro()
{
	float alt = Barometer_getAltitude(true);
	_plane->baro_alt = alt;
	_plane->baro_timestamp = get_time_us();
}
