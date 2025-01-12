#include "pitl_hal.h"

// ardupilot/libraries/SITL/SITL.h

Pitl_hal* Pitl_hal::_instance = nullptr;

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{
	_plane = plane;
	main_dt = 0.02;
	control_dt = 0.04;
	_instance = this;
}

void Pitl_hal::read_sensors()
{
	float data[10];

	if (buff1_ready)
	{
		int count = 0;
		char* token = strtok((char*)usb_buff1, ",");
		while (token != NULL)
		{
			data[count] = atof(token);
			count++;
			token = strtok(NULL, ",");
		}

		buff1_ready = false;
	}
	else if (buff2_ready)
	{
		int count = 0;
		char* token = strtok((char*)usb_buff2, ",");
		while (token != NULL)
		{
			data[count] = atof(token);
			count++;
			token = strtok(NULL, ",");
		}

		buff2_ready = false;
	}
	else
	{
		return;
	}

	printf("R:%.1f %.1f %.1f\n", _plane->ahrs_roll, _plane->ahrs_pitch, _plane->ahrs_yaw);

	uint64_t time = get_time_us();

	_plane->imu_gx = data[0];
	_plane->imu_gy = data[1];
	_plane->imu_gz = data[2];
	_plane->imu_ax = data[3];
	_plane->imu_ay = data[4];
	_plane->imu_az = data[5];
	_plane->imu_timestamp = time;

//	_plane->compass_mx = data[6];
//	_plane->compass_my = data[7];
//	_plane->compass_mz = data[8];
//	_plane->compass_timestamp = time;
//
//	_plane->baro_alt = data[9];
//	_plane->baro_timestamp = time;
//
//	_plane->gnss_lat = data[10];
//	_plane->gnss_lon = data[11];
//	_plane->gnss_asl = data[12];
//	_plane->gnss_sats = 10;
//	_plane->gnss_timestamp = time;

	// Transmit control commands
	char txBuf[100];
	sprintf(txBuf, "%f,%f\n", _rudder, _elevator);
	CDC_Transmit_FS((uint8_t*)txBuf, strlen(txBuf));
}

// Read sensor data from USB and add to plane struct
void Pitl_hal::usb_rx_callback(uint8_t* Buf, uint32_t Len)
{
	if (buff1_active)
	{
		for (uint32_t i = 0; i < Len; i++)
		{
			usb_buff1[i] = Buf[i];
		}

		buff1_ready = true;
	}
	else
	{
		for (uint32_t i = 0; i < Len; i++)
		{
			usb_buff2[i] = Buf[i];
		}

		buff2_ready = true;
	}

	buff1_active = !buff1_active;
}
