#include "Flight_HAL/Flight_hal.h"

void Flight_hal::usb_rx_callback(uint8_t* Buf, uint32_t Len)
{
	Hitl_rx_packet* p = (Hitl_rx_packet*)Buf;

	if (buff1_active)
	{
		usb_buff1 = p;
		buff1_ready = true;
	}
	else
	{
		usb_buff2 = p;
		buff2_ready = true;
	}

	buff1_active = !buff1_active;
}
