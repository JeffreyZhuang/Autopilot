#include "ahrs.h"

AHRS::AHRS(HAL* hal, Plane* plane)
	: filter(hal->get_main_dt()),
	  avg_ax(window_size, window_ax),
	  avg_ay(window_size, window_ay),
	  avg_az(window_size, window_az),
	  avg_mx(window_size, window_mx),
	  avg_my(window_size, window_my),
	  avg_mz(window_size, window_mz)
{
    _plane = plane;
    _hal = hal;
}

void AHRS::update()
{
	if (_plane->system_mode != System_mode::CONFIG)
	{
		filter.set_beta(get_params()->ahrs_beta);

		switch (ahrs_state)
		{
		case Ahrs_state::INITIALIZATION:
			update_initialization();
			break;
		case Ahrs_state::RUNNING:
			update_running();
			break;
		}
	}
}

void AHRS::update_initialization()
{
	if (check_new_imu_data() && check_new_compass_data())
	{
		float mag_data[3] = {_plane->compass_mx, _plane->compass_my, _plane->compass_mz};
		apply_compass_calibration(mag_data);

		avg_ax.add(-_plane->imu_ax);
		avg_ay.add(-_plane->imu_ay);
		avg_az.add(-_plane->imu_az);
		avg_mx.add(-mag_data[0]);
		avg_my.add(-mag_data[1]);
		avg_mz.add(-mag_data[2]);

		last_imu_timestamp = _plane->imu_timestamp;
		last_compass_timestamp = _plane->compass_timestamp;

		if (avg_ax.getFilled())
		{
			float q0, q1, q2, q3;
			float ax = avg_ax.getAverage();
			float ay = avg_ay.getAverage();
			float az = avg_az.getAverage();
			float mx = avg_mx.getAverage();
			float my = avg_my.getAverage();
			float mz = avg_mz.getAverage();

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

			ahrs_state = Ahrs_state::RUNNING;
		}
	}
}

void AHRS::update_running()
{
	if (check_new_imu_data())
	{
		last_imu_timestamp = _plane->imu_timestamp;
		if (is_accel_reliable())
		{
			if (check_new_compass_data())
			{
				last_compass_timestamp = _plane->compass_timestamp;
				update_imu_mag();
			}
			else
			{
				update_imu();
			}
		}
		else
		{
			update_gyro();
		}

		publish_ahrs();
	}
}

void AHRS::update_imu()
{
	filter.updateIMU(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
	                 -_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az);
}

void AHRS::update_imu_mag()
{
	float mag_data[3] = {_plane->compass_mx, _plane->compass_my, _plane->compass_mz};
	apply_compass_calibration(mag_data);

	filter.update(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
				  -_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az,
				  -mag_data[0], -mag_data[1], -mag_data[2]);
}

void AHRS::update_gyro()
{
	filter.updateGyro(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz);
}

void AHRS::publish_ahrs()
{
	_plane->ahrs_roll = filter.getRoll();
	_plane->ahrs_pitch = filter.getPitch();

	// Account for magnetic declination
	// Keep between 0 and 360 degrees
	_plane->ahrs_yaw = fmod(filter.getYaw() + get_params()->mag_decl + 360.0f, 360.0f);

	_plane->ahrs_timestamp = _hal->get_time_us();
	_plane->ahrs_converged = ahrs_state == Ahrs_state::RUNNING;
}

void AHRS::apply_compass_calibration(float mag_data[3])
{
	// Storage for hard-iron calibrated magnetometer data
	float hi_cal[3];

	// Apply hard-iron offsets
	hi_cal[0] = mag_data[0] - get_params()->hard_iron_x;
	hi_cal[1] = mag_data[1] - get_params()->hard_iron_y;
	hi_cal[2] = mag_data[2] - get_params()->hard_iron_z;

	// Apply soft-iron scaling
	mag_data[0] = (get_params()->soft_iron_xx * hi_cal[0]) +
				  (get_params()->soft_iron_xy * hi_cal[1]) +
				  (get_params()->soft_iron_xz * hi_cal[2]);
	mag_data[1] = (get_params()->soft_iron_yx * hi_cal[0]) +
				  (get_params()->soft_iron_yy * hi_cal[1]) +
				  (get_params()->soft_iron_yz * hi_cal[2]);
	mag_data[2] = (get_params()->soft_iron_zx * hi_cal[0]) +
				  (get_params()->soft_iron_zy * hi_cal[1]) +
				  (get_params()->soft_iron_zz * hi_cal[2]);
}

bool AHRS::is_accel_reliable()
{
	float accel_magnitude = sqrtf(powf(_plane->imu_ax, 2) +
								  powf(_plane->imu_ay, 2) +
								  powf(_plane->imu_az, 2));

	// Assuming 1g reference
	return fabs(accel_magnitude - 1.0f) < get_params()->ahrs_acc_max;
}

bool AHRS::check_new_imu_data()
{
    return _plane->imu_timestamp > last_imu_timestamp;
}

bool AHRS::check_new_compass_data()
{
    return _plane->compass_timestamp > last_compass_timestamp;
}
