#include "pitl_hal.h"

void Pitl_hal::transmit_telem(uint8_t tx_buff[], int len)
{
	mlrs_telem.transmit(tx_buff, len);
}

bool Pitl_hal::read_telem(uint8_t* rx_buff, uint8_t *size)
{
	return mlrs_telem.read(rx_buff, size);
}

void Pitl_hal::get_rc_input(uint16_t duty[], int size)
{
	mlrs_rc.get_rc_data(duty, size);
}
