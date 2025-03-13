#include "PITL_HAL/pitl_hal.h"

void Pitl_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	mlrs_telem.transmit(tx_buff, len);
}

bool Pitl_hal::read_telem(uint8_t rx_buff[], uint16_t *size)
{
	return mlrs_telem.read(rx_buff, size);
}

void Pitl_hal::get_rc_input(uint16_t duty[], uint8_t num_channels)
{
	mlrs_rc.get_rc_data(duty, num_channels);
}
