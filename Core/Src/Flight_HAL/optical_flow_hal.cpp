#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_of()
{
	cxof.setup();
}

bool Flight_hal::read_optical_flow(int16_t *x, int16_t *y)
{
	Cxof_frame result;
	if (cxof.read(&result))
	{
		*x = result.x;
		*y = result.y;

		return true;
	}

	return false;
}
