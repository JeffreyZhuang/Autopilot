#include "pitl_hal.h"

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{
	_plane = plane;
}

// Read AHRS and Navigation data from USB and add to plane struct
void Pitl_hal::read_sensors()
{
	uint8_t rxBuf[100];
//	USBD_CDC_SetRxBuffer();
}
