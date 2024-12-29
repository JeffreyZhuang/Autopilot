#include "derived_hal.h"

void Derived_hal::read_power_monitor()
{
	float voltage = ina219.read_voltage();
	float current = ina219.read_current();
	_plane->autopilot_voltage = voltage;
	_plane->autopilot_current = current;
}
