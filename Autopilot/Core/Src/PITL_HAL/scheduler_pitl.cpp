#include "pitl_hal.h"

void Pitl_hal::set_main_task(void (*task)())
{
	uint64_t prev_time = get_time_us();
	uint64_t dt = 1000000 / 100;

	while (1)
	{
		// Limit loop rate to 400Hz
		if (get_time_us() - prev_time > dt)
		{
			task();
		}
	}
}

// Read sensor data from USB and add to plane struct
void Pitl_hal::usb_rx_callback(uint8_t* Buf, uint32_t Len)
{
	printf("Received: %s\n", Buf);

	float data[10];
	int i = 0;
	char *token = (char*)Buf;
	while (token != NULL) {
		// Convert token to float and store in the array
		data[i] = strtof(token, NULL);
		i++;

		// Get the next token
		token = strtok(NULL, ",");
	}

	printf("%f\n", data[0]);

	// Parse
	uint64_t time = get_time_us();
	_plane->imu_timestamp = time;
	_plane->gnss_timestamp = time;
	_plane->compass_timestamp = time;
	_plane->baro_timestamp = time;

	// Transmit control commands
	char txBuf[100];
	sprintf(txBuf, "%f,%f\n", _elevator, _rudder);
	CDC_Transmit_FS((uint8_t*)txBuf, strlen(txBuf));
}

void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	Pitl_hal::get_instance()->usb_rx_callback(Buf, Len);
}


