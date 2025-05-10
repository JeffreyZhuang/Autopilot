#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::get_rc_input(uint16_t duty[], uint8_t num_channels)
{
	sbus_input.get_rc_data(duty, num_channels);
}
