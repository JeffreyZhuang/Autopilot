#include "pitl_hal.h"

void Pitl_hal::set_elevator(float deg)
{
	_elevator = -deg;
}

void Pitl_hal::set_rudder(float deg)
{
	_rudder = deg;
}
