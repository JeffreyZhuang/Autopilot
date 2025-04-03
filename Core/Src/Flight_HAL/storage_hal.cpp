#include "Flight_HAL/flight_hal.h"

void Flight_hal::create_file(char name[], uint8_t len)
{
	_sd.create_file(name, len);
}

bool Flight_hal::write_storage(uint8_t byte)
{
	return _sd.write_byte(byte);
}

bool Flight_hal::read_storage(uint8_t* rx_buff, uint16_t size)
{
	return _sd.read(rx_buff, size);
}
