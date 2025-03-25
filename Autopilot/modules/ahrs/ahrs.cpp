#include "modules/ahrs/ahrs.h"

AHRS::AHRS(HAL* hal, Plane* plane)
	: Module(hal, plane),
	  avg_ax(window_size, window_ax),
	  avg_ay(window_size, window_ay),
	  avg_az(window_size, window_az),
	  avg_mx(window_size, window_mx),
	  avg_my(window_size, window_my),
	  avg_mz(window_size, window_mz)
{
}

void AHRS::update()
{
	if (_plane->system_mode != Plane::System_mode::CONFIG)
	{
		filter.set_dt(_plane->dt_s);
		filter.set_beta(get_params()->ahrs.beta_gain);

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
	if (_plane->check_new_imu_data(imu_handle) && _plane->check_new_mag_data(mag_handle))
	{
		Plane::IMU_data imu_data = _plane->get_imu_data(imu_handle);
		Plane::Mag_data mag_data = _plane->get_mag_data(mag_handle);

		float mag_calib[3] = {mag_data.x, mag_data.y, mag_data.z};
		apply_compass_calibration(mag_calib);

		avg_ax.add(imu_data.ax);
		avg_ay.add(imu_data.ay);
		avg_az.add(imu_data.az);
		avg_mx.add(mag_calib[0]);
		avg_my.add(mag_calib[1]);
		avg_mz.add(mag_calib[2]);

		if (avg_ax.getFilled())
		{
			set_initial_angles();
			ahrs_state = Ahrs_state::RUNNING;
		}
	}
}

void AHRS::update_running()
{
	if (_plane->imu_data.check_new(imu_handle))
	{
		imu_data = _plane->imu_data.get(imu_handle);

		if (is_accel_reliable())
		{
			if (_plane->mag_data.check_new(mag_handle))
			{
				mag_data = _plane->mag_data.get(mag_handle);
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
	Plane::IMU_data imu_data = _plane->get_imu_data(imu_handle);

	filter.updateIMU(imu_data.gx, imu_data.gy, imu_data.gz,
	                 -imu_data.ax, -imu_data.ay, -imu_data.az);
}

void AHRS::update_imu_mag()
{
	Plane::IMU_data imu_data = _plane->get_imu_data(imu_handle);
	Plane::Mag_data mag_data = _plane->get_mag_data(mag_handle);

	float mag_calib[3] = {mag_data.x, mag_data.y, mag_data.z};

	apply_compass_calibration(mag_calib);

	filter.update(imu_data.gx, imu_data.gy, imu_data.gz,
				  -imu_data.ax, -imu_data.ay, -imu_data.az,
				  -mag_calib[0], -mag_calib[1], -mag_calib[2]);
}

void AHRS::update_gyro()
{
	Plane::IMU_data imu_data = _plane->get_imu_data(imu_handle);

	filter.updateGyro(imu_data.gx, imu_data.gy, imu_data.gz);
}

void AHRS::publish_ahrs()
{
	// Account yaw for magnetic declination and normalize to [0, 360]
	_plane->set_ahrs_data(Plane::AHRS_data{
		ahrs_state == Ahrs_state::RUNNING,
		filter.getRoll(),
		filter.getPitch(),
		fmod(filter.getYaw() + get_params()->ahrs.mag_decl + 360.0f, 360.0f),
		_hal->get_time_us()
	});
}

void AHRS::set_initial_angles()
{
	float q0, q1, q2, q3;
	float ax = -avg_ax.getAverage();
	float ay = -avg_ay.getAverage();
	float az = -avg_az.getAverage();
	float mx = -avg_mx.getAverage();
	float my = -avg_my.getAverage();
	float mz = -avg_mz.getAverage();

	float roll_initial = atan2f(ay, az);
	float pitch_initial = atan2f(-ax, sqrtf(powf(ay, 2) + powf(az, 2)));

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
		float yaw_initial = atan2f(-my, mx);

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
}

void AHRS::apply_compass_calibration(float mag_data[3])
{
	// Storage for hard-iron calibrated magnetometer data
	float hi_cal[3];

	// Apply hard-iron offsets
	hi_cal[0] = mag_data[0] - get_params()->ahrs.hard_iron_x;
	hi_cal[1] = mag_data[1] - get_params()->ahrs.hard_iron_y;
	hi_cal[2] = mag_data[2] - get_params()->ahrs.hard_iron_z;

	// Apply soft-iron scaling
	mag_data[0] = (get_params()->ahrs.soft_iron_xx * hi_cal[0]) +
				  (get_params()->ahrs.soft_iron_xy * hi_cal[1]) +
				  (get_params()->ahrs.soft_iron_xz * hi_cal[2]);
	mag_data[1] = (get_params()->ahrs.soft_iron_yx * hi_cal[0]) +
				  (get_params()->ahrs.soft_iron_yy * hi_cal[1]) +
				  (get_params()->ahrs.soft_iron_yz * hi_cal[2]);
	mag_data[2] = (get_params()->ahrs.soft_iron_zx * hi_cal[0]) +
				  (get_params()->ahrs.soft_iron_zy * hi_cal[1]) +
				  (get_params()->ahrs.soft_iron_zz * hi_cal[2]);
}

bool AHRS::is_accel_reliable()
{
	Plane::IMU_data imu_data = _plane->get_imu_data(imu_handle);

	float accel_magnitude = sqrtf(powf(imu_data.ax, 2) +
								  powf(imu_data.ay, 2) +
								  powf(imu_data.az, 2));

	// Assuming 1g reference
	return fabs(accel_magnitude - 1.0f) < get_params()->ahrs.acc_max;
}
