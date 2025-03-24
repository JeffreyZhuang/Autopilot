#include "Flight_HAL/Flight_hal.h"

void Flight_hal::get_rc_input(uint16_t duty[], uint8_t num_channels)
{
	sbus_input.get_rc_data(duty, num_channels);
}
