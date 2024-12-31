#include <flight_hal.h>

void Derived_hal::read_power_monitor()
{
	float voltage = _ina219.read_voltage();
	float current = _ina219.read_current();
	_plane->autopilot_voltage = voltage;
	_plane->autopilot_current = current;
}
