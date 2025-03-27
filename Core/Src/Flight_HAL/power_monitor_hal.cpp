#include "Flight_HAL/flight_hal.h"

void Flight_hal::read_power_monitor()
{
	float voltage = _ina219.read_voltage();
	float current = _ina219.read_current();
	_power_pub.publish(Power_data{0, 0, voltage, current, get_time_us()});
}
