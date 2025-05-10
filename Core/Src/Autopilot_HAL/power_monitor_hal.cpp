#include "Autopilot_HAL/Autopilot_HAL.h"

bool AutopilotHAL::read_power_monitor(float *voltage, float* current)
{
	*voltage = _ina219.read_voltage();
	*current = _ina219.read_current();
	return true;
}
