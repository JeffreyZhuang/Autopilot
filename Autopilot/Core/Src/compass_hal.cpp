#include "derived_hal.h"

void Derived_hal::init_compass()
{
	mag.begin_SPI(GPIOC, GPIO_PIN_13, &hspi1, SPI_BAUDRATEPRESCALER_8);
}

void Derived_hal::read_compass()
{
	mag.readDataNonBlocking();
}
