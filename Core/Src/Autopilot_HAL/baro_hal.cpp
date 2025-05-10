#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::init_baro()
{
	Barometer_init();
	Barometer_setOSR(OSR_4096);
}

bool AutopilotHAL::read_baro(float *alt)
{
	return Barometer_getAltitude(alt);
}
