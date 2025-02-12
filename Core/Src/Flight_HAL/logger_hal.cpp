#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_logger()
{
	_sd.initialize();
}

void Flight_hal::write_storage_buffer(uint8_t* packet, uint16_t len)
{
	_sd.append_buffer(packet, len);
}

void Flight_hal::flush_storage_buffer()
{
	_sd.write();
}

void Flight_hal::read_storage(uint8_t* rx_buff, uint16_t size)
{
	_sd.read(rx_buff, size);
}
