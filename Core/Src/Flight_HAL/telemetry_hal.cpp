#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_telem()
{
	sbus_input.setup();
	telem.setup();
}

void Flight_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	telem.transmit(tx_buff, len);
}

bool Flight_hal::read_telem(uint8_t* byte)
{
	return telem.read_byte(byte);
}

bool Flight_hal::telem_buffer_empty()
{
	return telem.buffer_empty();
}
