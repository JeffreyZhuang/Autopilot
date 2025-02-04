#include "pitl_hal.h"

// ardupilot/libraries/SITL/SITL.h

Pitl_hal* Pitl_hal::_instance = nullptr;

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane), mlrs_rc(&huart4), mlrs_telem(&huart6)
{
	_plane = plane;

	main_dt = 0.02;
	control_dt = 0.04;

	_instance = this;
}

void Pitl_hal::init()
{
	printf("Start\n");

	mlrs_rc.setup();
	mlrs_telem.setup();
}

void Pitl_hal::read_sensors()
{
	read_rc();
	read_pitl();
}

void Pitl_hal::read_pitl()
{
	Pitl_rx_packet* data;

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
	_plane->gnss_timestamp = time;

	// Transmit control commands
	uint8_t txBuf[sizeof(Pitl_tx_packet)];
	memcpy(txBuf, &pitl_tx_packet, sizeof(Pitl_tx_packet));
	CDC_Transmit_FS(txBuf, sizeof(txBuf));
}

// Read sensor data from USB and add to plane struct
void Pitl_hal::usb_rx_callback(uint8_t* Buf, uint32_t Len)
{
	Pitl_rx_packet* p = (Pitl_rx_packet*)Buf;

	if (buff1_active)
	{
		usb_buff1 = p;
		buff1_ready = true;
	}
	else
	{
		usb_buff2 = p;
		buff2_ready = true;
	}

	buff1_active = !buff1_active;
}
