#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::init_telem()
{
	sbus_input.setup();
	telem.setup();
}

void AutopilotHAL::transmit_telem(uint8_t tx_buff[], int len)
{
	telem.transmit(tx_buff, len);
}

bool AutopilotHAL::read_telem(uint8_t* byte)
{
	return telem.read_byte(byte);
}

bool AutopilotHAL::telem_buffer_empty()
{
	return telem.buffer_empty();
}
