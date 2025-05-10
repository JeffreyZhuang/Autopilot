#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::create_file(char name[], uint8_t len)
{
	_sd.create_file(name, len);
}

bool AutopilotHAL::write_storage(uint8_t byte)
{
	return _sd.write_byte(byte);
}
