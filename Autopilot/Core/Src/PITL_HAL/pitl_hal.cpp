#include "pitl_hal.h"

// ardupilot/libraries/SITL/SITL.h

Pitl_hal* Pitl_hal::_instance = nullptr;

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{
	_plane = plane;
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

//	printf("%f\n", data[2]);
	printf("R:%.1f %.1f %.1f\n", _plane->ahrs_roll, _plane->ahrs_pitch, _plane->ahrs_yaw);

	uint64_t time = get_time_us();

	_plane->imu_ax = data[0];
	_plane->imu_ay = data[1];
	_plane->imu_az = data[2];
	_plane->imu_gx = 0;
	_plane->imu_gy = 0;
	_plane->imu_gz = 0;
	_plane->imu_timestamp = time;

	_plane->gnss_lat = data[3];
	_plane->gnss_lon = data[4];
	_plane->gnss_sats = 10;
	_plane->gnss_timestamp = time;

//	_plane->compass_mx = 0;
//	_plane->compass_my = 0;
//	_plane->compass_mz = 0;
//	_plane->compass_timestamp = time;

	_plane->baro_alt = 0;
	_plane->baro_timestamp = time;

	// Transmit control commands
	char txBuf[100];
	sprintf(txBuf, "%f,%f\n", _elevator, _rudder);
	CDC_Transmit_FS((uint8_t*)txBuf, strlen(txBuf));
}

// Read sensor data from USB and add to plane struct
void Pitl_hal::usb_rx_callback(uint8_t* Buf, uint32_t Len)
{
	if (buff1_active)
	{
		for (int i = 0; i < Len; i++)
		{
			usb_buff1[i] = Buf[i];
		}

		buff1_ready = true;
	}
	else
	{
		for (int i = 0; i < Len; i++)
		{
			usb_buff2[i] = Buf[i];
		}

		buff2_ready = true;
	}

	buff1_active = !buff1_active;
}
