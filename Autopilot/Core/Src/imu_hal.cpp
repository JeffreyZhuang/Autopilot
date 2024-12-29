#include "derived_hal.h"

void Derived_hal::init_imu()
{
	_imu.begin();
}

void Derived_hal::read_imu()
{
	_imu.getAGT();
	_plane->imu_ax = -_imu.accX();
	_plane->imu_ay = -_imu.accY();
	_plane->imu_az = _imu.accZ();
	_plane->imu_gx = -_imu.gyrX();
	_plane->imu_gy = -_imu.gyrY();
	_plane->imu_gz = _imu.gyrZ();
	_plane->imu_temp = _imu.temp();
	_plane->imu_timestamp = get_time_us();
}
