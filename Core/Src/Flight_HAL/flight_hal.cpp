#include "Flight_HAL/flight_hal.h"

Flight_hal* Flight_hal::_instance = nullptr;

Flight_hal::Flight_hal(Data_bus* data_bus) :
	_imu(&hspi1, GPIOC, GPIO_PIN_15, SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_4),
	_ina219(&hi2c1, 0.01),
	_gnss(&huart3),
	sbus_input(&huart4),
	telem(&huart6),
	servo1(&htim3, TIM_CHANNEL_1),
	servo2(&htim2, TIM_CHANNEL_1),
	cxof(&huart2)
{
	_data_bus = data_bus;
	_instance = this;
}

void Flight_hal::init()
{
	printf("HAL init\n");

	init_imu();
	init_baro();
	init_compass();
	init_gnss();
	init_telem();
	init_servos();
	init_of();

	printf("HAL init done\n");
}
