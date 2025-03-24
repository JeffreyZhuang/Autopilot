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
	_plane->set_mag_data(Mag_data{data->mx, data->my, data->mz, time});
	_plane->set_baro_data(Baro_data{data->asl, time});
	_plane->set_gnss_data(GNSS_data{
		(double)data->lat * 1E-7,
		(double)data->lon * 1E-7,
		data->asl,
		10,
		true,
		time
	});
	_plane->set_of_data(OF_data{data->of_x, data->of_y, time});
}
