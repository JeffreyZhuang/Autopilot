#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_imu()
{
	_imu.begin();

	_imu.setAccelODR(ICM42688::odr100);
	_imu.setAccelFS(ICM42688::gpm4);

	_imu.setGyroODR(ICM42688::odr100);
	_imu.setGyroFS(ICM42688::dps500);
}

void Flight_hal::read_imu()
{
	if (_imu.getAGT() == 1)
	{
		_plane->set_imu_data(Plane::IMU_data{-_imu.gyrX(), -_imu.gyrY(), _imu.gyrZ(), -_imu.accX(), -_imu.accY(), _imu.accZ(), get_time_us()});
	}
}
