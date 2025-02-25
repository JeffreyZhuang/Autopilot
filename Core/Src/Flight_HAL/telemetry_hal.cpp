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

bool Flight_hal::read_telem(uint8_t rx_buff[], uint8_t* size)
{
	return mlrs_telem.read(rx_buff, size);
}

void Flight_hal::get_rc_input(uint16_t duty[], int size)
{
	mlrs_rc.get_rc_data(duty, size);
}
