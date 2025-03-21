#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_telem()
{
	mlrs_rc.setup();
	mlrs_telem.setup();
}

void Flight_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	mlrs_telem.transmit(tx_buff, len);
}

bool Flight_hal::read_telem(uint8_t* byte)
{
	return mlrs_telem.read_byte(byte);
}

bool Flight_hal::telem_buffer_empty()
{
	return mlrs_telem.buffer_empty();
}

void Flight_hal::get_rc_input(uint16_t duty[], uint8_t num_channels)
{
	mlrs_rc.get_rc_data(duty, num_channels);
}
