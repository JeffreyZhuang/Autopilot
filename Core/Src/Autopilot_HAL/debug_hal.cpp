#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::debug_print(char * str)
{
	printf(str);
}

void AutopilotHAL::toggle_led()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); // LED
}
