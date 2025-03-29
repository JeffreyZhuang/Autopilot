#include "Flight_HAL/flight_hal.h"

bool Flight_hal::read_power_monitor(float *voltage, float* current)
{
	*voltage = _ina219.read_voltage();
	*current = _ina219.read_current();
	return true;
}
