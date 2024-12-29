#include "derived_hal.h"

void Derived_hal::init_imu()
{
	imu.begin();
}

void Derived_hal::read_imu()
{
	imu.getAGT();
	_plane->imu_ax = -imu.accX();
	_plane->imu_ay = -imu.accY();
	_plane->imu_az = imu.accZ();
	_plane->imu_gx = -imu.gyrX();
	_plane->imu_gy = -imu.gyrY();
	_plane->imu_gz = imu.gyrZ();
	_plane->imu_temp = imu.temp();
	_plane->imu_timestamp = get_time_us();
}
