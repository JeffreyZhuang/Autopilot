#include "Flight_HAL/flight_hal.h"

// Add override keywords

Flight_hal* Flight_hal::_instance = nullptr; // Can I just move this to class?

Flight_hal::Flight_hal(Plane * plane) : HAL(plane),
										_imu(&hspi1, GPIOC, GPIO_PIN_15, SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_4),
										_ina219(&hi2c1, 0.01),
										_gnss(&huart3),
										mlrs_rc(&huart4),
										mlrs_telem(&huart6),
										servo_elevator(&htim3, TIM_CHANNEL_1),
										servo_aileron(&htim2, TIM_CHANNEL_1)
{
	_instance = this;
	_plane = plane;
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
}

void Flight_hal::read_sensors()
{
	read_imu();
	read_baro();
	read_compass();
	read_gnss();
	read_power_monitor();
	read_rc();
}
