#include <Modules/AHRS/ahrs.h>

AHRS::AHRS(HAL* hal, Plane* plane, float dt) : filter(dt, 1),
											   avg_ax(window_size, window_ax),
											   avg_ay(window_size, window_ay),
											   avg_az(window_size, window_az),
											   avg_mx(window_size, window_mx),
											   avg_my(window_size, window_my),
											   avg_mz(window_size, window_mz)
{
    _plane = plane;
    _hal = hal;
    _dt = dt;
}

void AHRS::setup()
{

}

// Moving average
bool AHRS::set_initial_state()
{
	if (check_new_imu_data() && check_new_compass_data() && !initial_state_set)
	{
		// Get heading from mag and roll, pitch from accel, convert to quaternion and put in here
		float q0, q1, q2, q3;
		float ax = -_plane->imu_ax;
		float ay = -_plane->imu_ay;
		float az = -_plane->imu_az;
		float mx = -_plane->compass_mx;
		float my = -_plane->compass_my;
		float mz = -_plane->compass_mz;

		avg_ax.add(ax);
		avg_ay.add(ay);
		avg_az.add(az);
		avg_mx.add(mx);
		avg_my.add(my);
		avg_mz.add(mz);

		if (avg_ax.getFilled())
		{
			ax = avg_ax.getAverage();
			ay = avg_ay.getAverage();
			az = avg_az.getAverage();
			mx = avg_mx.getAverage();
			my = avg_my.getAverage();
			mz = avg_mz.getAverage();

			float roll_initial = atan2f(ay, az);
			float pitch_initial = atan2f(-ax, sqrtf(powf(ay, 2) + powf(az, 2)));
			float yaw_initial = 0;
			float norm = sqrtf(powf(mx, 2) + powf(my, 2) + powf(mz, 2));
			if (norm == 0)
			{
				q0 = 1.0f;
				q1 = 0.0f;
				q2 = 0.0f;
				q3 = 0.0f;
			}
			else
			{
				mx /= norm;
				my /= norm;
				mz /= norm;

				mx = mx * cosf(pitch_initial) + mz * sinf(pitch_initial);
				my = mx * sinf(roll_initial) * sin(pitch_initial) + my * cos(roll_initial) - mz * sinf(roll_initial) * cosf(pitch_initial);
				yaw_initial = atan2f(-my, mx);

				float cy = cosf(yaw_initial * 0.5f);
				float sy = sinf(yaw_initial * 0.5f);
				float cp = cosf(pitch_initial * 0.5f);
				float sp = sinf(pitch_initial * 0.5f);
				float cr = cosf(roll_initial * 0.5f);
				float sr = sinf(roll_initial * 0.5f);

				q0 = cr * cp * cy + sr * sp * sy;
				q1 = sr * cp * cy - cr * sp * sy;
				q2 = cr * sp * cy + sr * cp * sy;
				q3 = cr * cp * sy - sr * sp * cy;
			}

			filter.set_state(q0, q1, q2, q3); // Set initial state

			last_imu_timestamp = _plane->imu_timestamp;
			last_compass_timestamp = _plane->compass_timestamp;

			publish_ahrs();

			initial_state_set = true;

			return true;
		}
	}

	return false;
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
