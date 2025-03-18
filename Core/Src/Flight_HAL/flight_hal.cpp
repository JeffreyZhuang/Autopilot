#include "Flight_HAL/flight_hal.h"

Flight_hal* Flight_hal::_instance = nullptr;

Flight_hal::Flight_hal(Plane * plane) :
	_imu(&hspi1, GPIOC, GPIO_PIN_15, SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_4),
	_ina219(&hi2c1, 0.01),
	_gnss(&huart3),
	mlrs_rc(&huart4),
	mlrs_telem(&huart6),
	servo1(&htim3, TIM_CHANNEL_1),
	servo2(&htim2, TIM_CHANNEL_1),
	cxof(&huart2)
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

void Flight_hal::read_sensors_hitl()
{
	Hitl_rx_packet* data;

	if (buff1_ready)
	{
		data = usb_buff1;
		buff1_ready = false;
	}
	else if (buff2_ready)
	{
		data = usb_buff2;
		buff2_ready = false;
	}
	else
	{
		return;
	}

	uint64_t time = get_time_us();
	_plane->imu_gx = data->gx;
	_plane->imu_gy = data->gy;
	_plane->imu_gz = data->gz;
	_plane->imu_ax = data->ax;
	_plane->imu_ay = data->ay;
	_plane->imu_az = data->az;
	_plane->imu_timestamp = time;
	_plane->compass_mx = data->mx;
	_plane->compass_my = data->my;
	_plane->compass_mz = data->mz;
	_plane->compass_timestamp = time;
	_plane->baro_alt = data->asl;
	_plane->baro_timestamp = time;
	_plane->gnss_lat = data->lat;
	_plane->gnss_lon = data->lon;
	_plane->gnss_asl = data->asl;
	_plane->gnss_sats = 10;
	_plane->gps_fix = true;
	_plane->gnss_timestamp = time;
}
