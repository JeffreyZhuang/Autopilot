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
		_plane->imu_ax = -_imu.accX();
		_plane->imu_ay = -_imu.accY();
		_plane->imu_az = _imu.accZ();
		_plane->imu_gx = -_imu.gyrX();
		_plane->imu_gy = -_imu.gyrY();
		_plane->imu_gz = _imu.gyrZ();
		_plane->imu_temp = _imu.temp();
		_plane->imu_timestamp = get_time_us();
	}
}
