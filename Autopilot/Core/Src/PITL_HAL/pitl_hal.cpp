#include "pitl_hal.h"

Pitl_hal::Pitl_hal(Plane* plane) : HAL(plane)
{
	_plane = plane;
}

// Read sensor data from USB and add to plane struct
void Pitl_hal::read_sensors()
{
	uint8_t rxBuf[100];
	// USBD_CDC_SetRxBuffer();

	_plane->imu_ax = 0;
	_plane->imu_ay = 0;
	_plane->imu_az = 0;
	_plane->imu_gx = 0;
	_plane->imu_gy = 0;
	_plane->imu_gz = 0;
	_plane->imu_timestamp = get_time_us();

	// ardupilot/libraries/SITL/SITL.h
}

// Transmit USB
void Pitl_hal::set_elevator(float deg)
{
//	char txBuf[100];
//	CDC_Transmit_FS((uint8_t*)txBuf, strlen(txBuf));
}

// Transmit USB
void Pitl_hal::set_rudder(float deg)
{

}
