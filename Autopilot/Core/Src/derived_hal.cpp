#include "derived_hal.h"

Derived_hal::Derived_hal(Plane * plane) : HAL(plane),
										  imu(&hspi1, GPIOC, GPIO_PIN_15, SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_4),
										  ina219(&hi2c1, 0.01),
										  gnss(&huart3)
{
	_plane = plane;
}

void Derived_hal::init()
{
	init_imu();
	init_baro();

	mag.begin_SPI(GPIOC, GPIO_PIN_13, &hspi1, SPI_BAUDRATEPRESCALER_8);

	sd.initialize();

	gnss.setup();

	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
}

void Derived_hal::read_sensors()
{
	uint32_t time = HAL_GetTick();

	read_imu();
	read_baro();

	float voltage = ina219.read_voltage();
	float current = ina219.read_current();
	_plane->autopilot_voltage = voltage;
	_plane->autopilot_current = current;

	mag.readDataNonBlocking();

	uint8_t sentence[100];
	if (gnss.parse(sentence))
	{
		_plane->lat = gnss.lat;
		_plane->lon = gnss.lon;
	}
}

void Derived_hal::gnss_dma_complete()
{
	gnss.dma_complete();
}
