#include "pitl_hal.h"

// ardupilot/libraries/SITL/SITL.h

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{
	_plane = plane;
}

void Pitl_hal::read_sensors()
{
	// Transmit control commands
	char txBuf[100];
	sprintf(txBuf, "%f,%f\n", _elevator, _rudder);
	CDC_Transmit_FS((uint8_t*)txBuf, strlen(txBuf));
}
