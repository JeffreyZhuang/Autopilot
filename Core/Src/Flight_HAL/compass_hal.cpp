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
		float _hard_iron[3] = {52.67, -5.27, 81.54};
		float _soft_iron[3][3] = {{1.031, 0.015, -0.0032},
								  {0.015, 0.967, -0.025},
								  {-0.032, -0.025, 1.005}};
		float mag_data[3] = {_mag.x, _mag.y, _mag.z};
		float hi_cal[3];

		// Apply hard-iron offsets
		for (uint8_t i = 0; i < 3; i++) {
			hi_cal[i] = mag_data[i] - _hard_iron[i];
		}

		// Apply soft-iron scaling
		for (uint8_t i = 0; i < 3; i++) {
			mag_data[i] = (_soft_iron[i][0] * hi_cal[0]) +
						  (_soft_iron[i][1] * hi_cal[1]) +
						  (_soft_iron[i][2] * hi_cal[2]);
		}

		_plane->compass_mx = -mag_data[0];
		_plane->compass_my = mag_data[1];
		_plane->compass_mz = -mag_data[2];
		_plane->compass_timestamp = get_time_us();
	}
}
