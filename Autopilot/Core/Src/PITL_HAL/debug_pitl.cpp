#include "pitl_hal.h"

void Pitl_hal::debug_print(char * str)
{
	printf(str);
}

void Pitl_hal::toggle_led()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); // LED
}
