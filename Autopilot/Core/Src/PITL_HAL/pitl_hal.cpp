#include "pitl_hal.h"

// ardupilot/libraries/SITL/SITL.h

Pitl_hal* Pitl_hal::_instance = nullptr;

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{
	_plane = plane;
	_instance = this;
}

void Pitl_hal::xitl_run()
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

	uint64_t time = get_time_us();

	_plane->ahrs_roll = data[0];
	_plane->ahrs_pitch = data[1];
	_plane->ahrs_yaw = data[2];
	_plane->ahrs_timestamp = time;

	// How to put into nav?
	_plane->nav_timestamp = time;

	printf("R:%.1f %.1f %.1f\n", _plane->ahrs_roll, _plane->ahrs_pitch, _plane->ahrs_yaw);

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
