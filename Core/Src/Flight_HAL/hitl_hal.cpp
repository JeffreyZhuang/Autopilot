#include "Flight_HAL/Flight_hal.h"

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
	_plane->gnss_lat = (double)data->lat * 1E-7;
	_plane->gnss_lon = (double)data->lon * 1E-7;
	_plane->gnss_asl = data->asl;
	_plane->gnss_sats = 10;
	_plane->gps_fix = true;
	_plane->gnss_timestamp = time;
	_plane->of_x = data->of_x;
	_plane->of_y = data->of_y;
	_plane->of_timestamp = time;
}
