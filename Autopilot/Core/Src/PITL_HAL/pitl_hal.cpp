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
	// The only thing you have to simulate is the IMU, nothing else is hard.
	// Chatgpt: Write a function in python to convert accelerations in the world reference frame to IMU values
}

// Transmit USB
void Pitl_hal::set_elevator(float deg)
{

}

// Transmit USB
void Pitl_hal::set_rudder(float deg)
{

}
