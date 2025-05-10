#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::usb_transmit(uint8_t buf[], int len)
{
	usb_stream.transmit(buf, len);
}

bool AutopilotHAL::usb_read(uint8_t *byte)
{
	return usb_stream.read_byte(byte);
}

bool AutopilotHAL::usb_buffer_empty()
{
	return usb_stream.buffer_empty();
}
