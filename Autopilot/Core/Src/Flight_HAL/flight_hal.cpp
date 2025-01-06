#include "Flight_HAL/flight_hal.h"

void Flight_hal::init()
{
	init_imu();
	init_baro();
	init_compass();
	init_gnss();
	init_logger();
}

void Flight_hal::read_sensors()
{
	read_imu();
	read_baro();
	read_compass();
	read_gnss();
	read_power_monitor();
}
