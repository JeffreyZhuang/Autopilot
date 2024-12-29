#include "derived_hal.h"

void Derived_hal::debug_print(char * str)
{
	printf(str);
}

void Derived_hal::usb_print(char * str)
{
	CDC_Transmit_FS((uint8_t*)str, strlen(str));
}

void Derived_hal::toggle_led()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); // LED
}
