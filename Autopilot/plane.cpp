#include "plane.h"

// Check if the IMU data is outdated
bool Plane::check_new_imu_data(IMU_data imu_data)
{
	return _imu_data.timestamp > imu_data.timestamp;
}

IMU_data Plane::get_imu_data()
{
	return _imu_data;
}

void Plane::set_imu_data(IMU_data imu_data)
{
	_imu_data = imu_data;
}

bool Plane::check_new_of_data(OF_data of_data)
{
	return _of_data.timestamp > of_data.timestamp;
}

OF_data Plane::get_of_data()
{
	return _of_data;
}

void Plane::set_of_data(OF_data of_data)
{
	_of_data = of_data;
}
