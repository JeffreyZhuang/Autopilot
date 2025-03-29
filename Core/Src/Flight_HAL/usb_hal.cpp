#include "Flight_HAL/Flight_hal.h"

void Flight_hal::usb_transmit(uint8_t buf[], int len)
{
	usb_stream.transmit(buf, len);
}

bool Flight_hal::usb_read(uint8_t *byte)
{
	return usb_stream.read_byte(byte);
}
