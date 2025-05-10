#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::init_gnss()
{
	_gnss.setup();
}

bool AutopilotHAL::read_gnss(double *lat, double *lon, float* alt, uint8_t* sats, bool* fix)
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
