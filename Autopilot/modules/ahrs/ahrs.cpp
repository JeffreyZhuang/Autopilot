#include "modules/ahrs/ahrs.h"

AHRS::AHRS(HAL* hal, DataBus* data_bus)
	: Module(hal, data_bus),
	  avg_ax(window_size, window_ax),
	  avg_ay(window_size, window_ay),
	  avg_az(window_size, window_az),
	  avg_mx(window_size, window_mx),
	  avg_my(window_size, window_my),
	  avg_mz(window_size, window_mz),
	  _modes_sub(data_bus->modes_node),
	  _mag_sub(data_bus->mag_node),
	  _imu_sub(data_bus->imu_node),
	  _ahrs_pub(data_bus->ahrs_node)
{
}

void AHRS::parameters_update()
{
	param_get(AHRS_MAG_DECL, &_mag_decl);
	param_get(AHRS_BETA_GAIN, &_ahrs_beta_gain);
	param_get(AHRS_ACC_MAX, &_ahrs_acc_max);

	filter.set_dt(_dt);
	filter.set_beta(_ahrs_beta_gain);
}

void AHRS::update()
{
	parameters_update();

	const uint64_t time = _hal->get_time_us();
	_dt = clamp((time - _last_time) * US_TO_S, DT_MIN, DT_MAX);
	_last_time = time;

	_modes_data = _modes_sub.get();

	if (_modes_data.system_mode != System_mode::LOAD_PARAMS)
	{
		if (!_ahrs_data.converged)
		{
			update_initialization();
		}
		else
		{
			update_running();
		}
	}
}

void AHRS::update_initialization()
{
	if (_imu_sub.check_new() && _mag_sub.check_new())
	{
		_imu_data = _imu_sub.get();
		_mag_data = _mag_sub.get();

		avg_ax.add(_imu_data.ax);
		avg_ay.add(_imu_data.ay);
		avg_az.add(_imu_data.az);
		avg_mx.add(_mag_data.x);
		avg_my.add(_mag_data.y);
		avg_mz.add(_mag_data.z);

		if (avg_ax.getFilled())
		{
			set_initial_angles();

			_ahrs_data.converged = true;
		}
	}
}

void AHRS::update_running()
{
	if (_imu_sub.check_new())
	{
		_imu_data = _imu_sub.get();

		if (is_accel_reliable())
		{
			if (_mag_sub.check_new())
			{
				_mag_data = _mag_sub.get();
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
	filter.updateIMU(_imu_data.gx, _imu_data.gy, _imu_data.gz,
	                 -_imu_data.ax, -_imu_data.ay, -_imu_data.az);
}

void AHRS::update_imu_mag()
{
	filter.update(_imu_data.gx, _imu_data.gy, _imu_data.gz,
				  -_imu_data.ax, -_imu_data.ay, -_imu_data.az,
				  -_mag_data.x, -_mag_data.y, -_mag_data.z);
}

void AHRS::update_gyro()
{
	filter.updateGyro(_imu_data.gx, _imu_data.gy, _imu_data.gz);
}

void AHRS::publish_ahrs()
{
	_ahrs_data.roll = filter.getRoll();
	_ahrs_data.pitch = filter.getPitch();

	// Add magnetic declination and normalize to [-180, 180]
	_ahrs_data.yaw = fmod(filter.getYaw() + _mag_decl + 180.0f, 360.0f) - 180.0f;

	_ahrs_data.timestamp = _hal->get_time_us();

	_ahrs_pub.publish(_ahrs_data);
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

	printf("AHRS Initialized: %.01f %.01f %.01f\n", filter.getRoll(), filter.getPitch(), filter.getYaw());
}

bool AHRS::is_accel_reliable()
{
	float accel_magnitude = sqrtf(powf(_imu_data.ax, 2) +
								  powf(_imu_data.ay, 2) +
								  powf(_imu_data.az, 2));

	// Assuming 1g reference
	return fabs(accel_magnitude - 1.0f) < _ahrs_acc_max;
}
