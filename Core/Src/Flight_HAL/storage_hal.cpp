#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_logger()
{
	_sd.initialize();
}

void Flight_hal::write_storage_buffer(uint8_t* packet, uint16_t len)
{
	_sd.write(packet, len);
}

bool Flight_hal::read_storage(uint8_t* rx_buff, uint16_t size)
{
	return _sd.read(rx_buff, size);
}
