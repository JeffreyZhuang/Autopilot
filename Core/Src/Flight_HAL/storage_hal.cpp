#include "Flight_HAL/flight_hal.h"

void Flight_hal::create_file(char name[], uint8_t len)
{
	_sd.create_file(name, len);
}

bool Flight_hal::write_storage(uint8_t byte)
{
	return _sd.write_byte(byte);
}
