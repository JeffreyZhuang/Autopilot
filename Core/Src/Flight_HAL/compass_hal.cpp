#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_compass()
{
	_mag.begin_SPI(GPIOC, GPIO_PIN_13, &hspi1, SPI_BAUDRATEPRESCALER_8);
	_mag.setFilter(MLX90393_FILTER_3);
	_mag.setOversampling(MLX90393_OSR_2);
}

void Flight_hal::read_compass()
{
	if (_mag.readDataNonBlocking())
	{
		_plane->set_mag_data(Mag_data{-_mag.x, _mag.y, -_mag.z, get_time_us()});
	}
}
