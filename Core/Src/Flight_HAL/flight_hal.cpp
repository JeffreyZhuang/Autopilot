#include "Flight_HAL/flight_hal.h"

Flight_hal* Flight_hal::_instance = nullptr;

Flight_hal::Flight_hal() :
	_imu(&hspi1, GPIOC, GPIO_PIN_15, SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_4),
	_ina219(&hi2c1, 0.01),
	_gnss(&huart3),
	sbus_input(&huart4),
	telem(&huart6),
	servo1(&htim3, TIM_CHANNEL_1),
	servo2(&htim2, TIM_CHANNEL_1),
	cxof(&huart2),
	_imu_pub(Data_bus::get_instance().imu_node),
	_mag_pub(Data_bus::get_instance().mag_node),
	_baro_pub(Data_bus::get_instance().baro_node),
	_of_pub(Data_bus::get_instance().of_node),
	_gnss_pub(Data_bus::get_instance().gnss_node),
	_power_pub(Data_bus::get_instance().power_node)
{
	_instance = this;
}

void Flight_hal::init()
{
	init_imu();
	init_baro();
	init_compass();
	init_gnss();
	init_logger();
	init_telem();
	init_servos();
	init_of();
}

void Flight_hal::read_sensors_flight()
{
	read_imu();
	read_baro();
	read_compass();
	read_gnss();
	read_power_monitor();
	read_of();
}
