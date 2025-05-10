#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::init_of()
{
	cxof.setup();
}

bool AutopilotHAL::read_optical_flow(int16_t *x, int16_t *y)
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
