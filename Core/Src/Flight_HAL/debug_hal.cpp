#include "Flight_HAL/flight_hal.h"

void Flight_hal::debug_print(char * str)
{
	printf(str);
}

void Flight_hal::toggle_led()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); // LED
}
