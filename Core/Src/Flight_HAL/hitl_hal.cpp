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
	_plane->set_imu_data(IMU_data{data->gx, data->gy, data->gz, data->ax, data->ay, data->az, time});
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
	_plane->set_of_data(OF_data{data->of_x, data->of_y, time});
}
