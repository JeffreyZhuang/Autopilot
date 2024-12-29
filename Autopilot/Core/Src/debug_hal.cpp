#include "derived_hal.h"

void Derived_hal::swo_print(char * str)
{

}

void Derived_hal::usb_print(char * str)
{

}

void Derived_hal::i2c_scan()
{

}

void Derived_hal::toggle_led()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); // LED
}
