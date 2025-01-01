#include "pitl_hal.h"

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{

}

void Pitl_hal::init()
{

}

void Pitl_hal::read_sensors()
{
	// Instead of setting sensor values of plane struct, set AHRS and navigation values from USB
}
