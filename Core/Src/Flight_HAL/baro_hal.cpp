#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_baro()
{
	Barometer_init();
	Barometer_setOSR(OSR_4096);
}

bool Flight_hal::read_baro(float *alt)
{
	return Barometer_getAltitude(alt);
}
