#include <Modules/AHRS/ahrs.h>

AHRS::AHRS(HAL* hal, Plane* plane, float dt) : mekf(sigma_a, sigma_g, sigma_m)
{
    _plane = plane;
    _hal = hal;
    _dt = dt;
}

void AHRS::setup()
{
//	updateEKFQuatAtt_initialize();
}

bool AHRS::set_initial_state()
{
	if (check_new_imu_data() && check_new_compass_data() && !initial_state_set)
	{
		Eigen::Vector3f acc0 = {-_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az};
		Eigen::Vector3f mag0 = {-_plane->compass_mx, -_plane->compass_my, -_plane->compass_mz};
		mekf.initialize_from_acc_mag(acc0, mag0);

		initial_state_set = true;

		return true;
	}

	return false;
}
//	if (check_new_imu_data() && check_new_compass_data() && !initial_state_set)
//	{
//		filter.initWithAcc(_plane->imu_ax, _plane->imu_ay, _plane->imu_az);
//
//		initial_state_set = true;
//
//		return true;
//	}
//
//	return false;
//}

//	if (check_new_imu_data() && check_new_compass_data() && !initial_state_set)
//	{
//		// Get heading from mag and roll, pitch from accel, convert to quaternion and put in here
//		float q0, q1, q2, q3;
//		float ax = -_plane->imu_ax;
//		float ay = -_plane->imu_ay;
//		float az = -_plane->imu_az;
//		float mx = -_plane->compass_mx;
//		float my = -_plane->compass_my;
//		float mz = -_plane->compass_mz;
//
//		float roll_initial = atan2f(ay, az);
//		float pitch_initial = atan2f(-ax, sqrtf(powf(ay, 2) + powf(az, 2)));
//		float yaw_initial = 0;
//		float norm = sqrtf(powf(mx, 2) + powf(my, 2) + powf(mz, 2));
//		if (norm == 0)
//		{
//			q0 = 1.0f;
//			q1 = 0.0f;
//			q2 = 0.0f;
//			q3 = 0.0f;
//		}
//		else
//		{
//			mx /= norm;
//			my /= norm;
//			mz /= norm;
//
//			mx = mx * cosf(pitch_initial) + mz * sinf(pitch_initial);
//			my = mx * sinf(roll_initial) * sin(pitch_initial) + my * cos(roll_initial) - mz * sinf(roll_initial) * cosf(pitch_initial);
//			yaw_initial = atan2f(-my, mx);
//
//			float cy = cosf(yaw_initial * 0.5f);
//			float sy = sinf(yaw_initial * 0.5f);
//			float cp = cosf(pitch_initial * 0.5f);
//			float sp = sinf(pitch_initial * 0.5f);
//			float cr = cosf(roll_initial * 0.5f);
//			float sr = sinf(roll_initial * 0.5f);
//
//			q0 = cr * cp * cy + sr * sp * sy;
//			q1 = sr * cp * cy - cr * sp * sy;
//			q2 = cr * sp * cy + sr * cp * sy;
//			q3 = cr * cp * sy - sr * sp * cy;
//		}
//
//		filter.set_state(q0, q1, q2, q3); // Set initial state
//
//		last_imu_timestamp = _plane->imu_timestamp;
//		last_compass_timestamp = _plane->compass_timestamp;
//
//		publish_ahrs();
//
//		initial_state_set = true;
//
//		return true;
//	}
//
//	return false;
//}

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
	Eigen::Vector3f gyr = {_plane->imu_gx * M_PI / 180.0f, _plane->imu_gy * M_PI / 180.0f, _plane->imu_gz * M_PI / 180.0f};
	Eigen::Vector3f acc = {-_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az};
	Eigen::Vector3f mag = {-_plane->compass_mx, -_plane->compass_my, -_plane->compass_mz};

	mekf.time_update(gyr, _dt);
	mekf.measurement_update(acc, mag);

	Eigen::Vector4f quat = mekf.quaternion();
	float q0 = quat(0);
	float q1 = quat(1);
	float q2 = quat(2);
	float q3 = quat(3);
	float roll_deg = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 180.0f / M_PI;
	float pitch_deg = asinf(-2.0f * (q1*q3 - q0*q2)) * 180.0f / M_PI;
	float yaw_deg = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 180.0f / M_PI;
	printf("%f %f %f\n", yaw_deg, pitch_deg, roll_deg);

	_plane->ahrs_roll = yaw_deg;
	_plane->ahrs_pitch = pitch_deg;
	_plane->ahrs_yaw = roll_deg;


//	float roll_deg, pitch_deg, yaw_deg;
//	const float gyro[3] = {_plane->imu_gx * M_PI / 180.0f, _plane->imu_gy * M_PI / 180.0f, _plane->imu_gz * M_PI / 180.0f};
//	const float accel[3] = {_plane->imu_ax * 9.81, _plane->imu_ay * 9.81, _plane->imu_az * 9.81};
//	const float mag[3] = {_plane->compass_mx, _plane->compass_my, _plane->compass_mz};
//	updateEKFQuatAtt(gyro, accel, mag, 0.0f, 0.0f, _dt, 1.0f, &roll_deg, &pitch_deg, &yaw_deg);
//
//	_plane->ahrs_roll = roll_deg;
//	_plane->ahrs_pitch = pitch_deg;
//	_plane->ahrs_yaw = yaw_deg;
//
//	printf("%f %f %f\n", roll_deg, pitch_deg, yaw_deg);

//	if (check_new_imu_data())
//	{
//		if (check_new_compass_data())
//		{
//			update_imu_mag();
//		}
//		else
//		{
//			update_imu();
//		}
//
//		publish_ahrs();
//	}
}

void AHRS::update_imu()
{
//	filter.updateIMU(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
//	                 -_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az);
//	last_imu_timestamp = _plane->imu_timestamp;
}

void AHRS::update_imu_mag()
{
//	filter.update(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
//				  -_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az,
//				  -_plane->compass_mx, -_plane->compass_my, -_plane->compass_mz);
//	last_imu_timestamp = _plane->imu_timestamp;
//	last_compass_timestamp = _plane->compass_timestamp;
}

void AHRS::publish_ahrs()
{
//	_plane->ahrs_roll = filter.getRoll();
//	_plane->ahrs_pitch = filter.getPitch();
//	_plane->ahrs_yaw = filter.getYaw();
//	_plane->ahrs_q0 = filter.get_q0();
//	_plane->ahrs_q1 = filter.get_q1();
//	_plane->ahrs_q2 = filter.get_q2();
//	_plane->ahrs_q3 = filter.get_q3();
//	_plane->ahrs_timestamp = _hal->get_time_us();
}
