#include "plane.h"

bool Plane::check_new_imu_data(Subscription_handle subscription_handle)
{
	return _imu_data.timestamp > subscription_handle.timestamp;
}

IMU_data Plane::get_imu_data(Subscription_handle subscription_handle)
{
	subscription_handle.timestamp = _imu_data.timestamp;
	return _imu_data;
}

void Plane::set_imu_data(IMU_data imu_data)
{
	_imu_data = imu_data;
}

bool Plane::check_new_mag_data(Subscription_handle subscription_handle)
{
	return _mag_data.timestamp > subscription_handle.timestamp;
}

Mag_data Plane::get_mag_data(Subscription_handle subscription_handle)
{
	subscription_handle.timestamp = _mag_data.timestamp;
	return _mag_data;
}

void Plane::set_mag_data(Mag_data mag_data)
{
	_mag_data = mag_data;
}

bool Plane::check_new_baro_data(Subscription_handle subscription_handle)
{
	return _baro_data.timestamp > subscription_handle.timestamp;
}

Baro_data Plane::get_baro_data(Subscription_handle subscription_handle)
{
	subscription_handle.timestamp = _baro_data.timestamp;
	return _baro_data;
}

void Plane::set_baro_data(Baro_data baro_data)
{
	_baro_data = baro_data;
}

bool Plane::check_new_gnss_data(Subscription_handle subscription_handle)
{
	return _gnss_data.timestamp > subscription_handle.timestamp;
}

GNSS_data Plane::get_gnss_data(Subscription_handle subscription_handle)
{
	subscription_handle.timestamp = _gnss_data.timestamp;
	return _gnss_data;
}

void Plane::set_gnss_data(GNSS_data gnss_data)
{
	_gnss_data = gnss_data;
}

bool Plane::check_new_of_data(Subscription_handle subscription_handle)
{
	return _of_data.timestamp > subscription_handle.timestamp;
}

OF_data Plane::get_of_data(Subscription_handle subscription_handle)
{
	subscription_handle.timestamp = _of_data.timestamp;
	return _of_data;
}

void Plane::set_of_data(OF_data of_data)
{
	_of_data = of_data;
}

bool Plane::check_new_ahrs_data(Subscription_handle subscription_handle)
{
	return _ahrs_data.timestamp > subscription_handle.timestamp;
}

AHRS_data Plane::get_ahrs_data(Subscription_handle subscription_handle)
{
	subscription_handle.timestamp = _ahrs_data.timestamp;
	return _ahrs_data;
}

void Plane::set_ahrs_data(AHRS_data ahrs_data)
{
	_ahrs_data = ahrs_data;
}

double Plane::get_home_lat() const
{
	return waypoints[0].lat;
}

double Plane::get_home_lon() const
{
	return waypoints[0].lon;
}
