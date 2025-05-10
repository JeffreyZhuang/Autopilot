#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::init_compass()
{
	_mag.begin_SPI(GPIOC, GPIO_PIN_13, &hspi1, SPI_BAUDRATEPRESCALER_8);
	_mag.setFilter(MLX90393_FILTER_3);
	_mag.setOversampling(MLX90393_OSR_2);
}

bool AutopilotHAL::read_mag(float *mx, float *my, float *mz)
{
	if (_mag.readDataNonBlocking())
	{
		*mx = -_mag.x;
		*my = _mag.y;
		*mz = -_mag.z;

		return true;
	}

	return false;
}
