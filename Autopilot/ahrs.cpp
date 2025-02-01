#include <ahrs.h>

AHRS::AHRS(HAL* hal, Plane* plane, float dt)
{
    _plane = plane;
    _hal = hal;
    _dt = dt;
}

void AHRS::setup()
{
	filter.begin(1.0f / _dt);
	filter.set_gain(0.1);
}

void AHRS::set_state()
{

}

bool AHRS::check_new_imu_data()
{
    return _plane->imu_timestamp != last_imu_timestamp;
}

bool AHRS::check_new_compass_data()
{
    return _plane->compass_timestamp != last_compass_timestamp;
}

void AHRS::apply_compass_calibration()
{

}

void AHRS::update()
{
	if (check_new_imu_data())
	{
		if (check_new_compass_data())
		{
			update_imu_mag();
		}
		else
		{
			update_imu();
		}

		publish_ahrs();
	}
}

void AHRS::update_imu()
{
	filter.updateIMU(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
	                 -_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az);
	last_imu_timestamp = _plane->imu_timestamp;
}

void AHRS::update_imu_mag()
{
	filter.update(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
				  -_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az,
				  -_plane->compass_mx, -_plane->compass_my, -_plane->compass_mz);
	last_imu_timestamp = _plane->imu_timestamp;
	last_compass_timestamp = _plane->compass_timestamp;
}

void AHRS::publish_ahrs()
{
	_plane->ahrs_roll = filter.getRoll();
	_plane->ahrs_pitch = filter.getPitch();
	_plane->ahrs_yaw = filter.getYaw();
	_plane->ahrs_q0 = filter.get_q0();
	_plane->ahrs_q1 = filter.get_q1();
	_plane->ahrs_q2 = filter.get_q2();
	_plane->ahrs_q3 = filter.get_q3();
	_plane->ahrs_timestamp = _hal->get_time_us();
}
