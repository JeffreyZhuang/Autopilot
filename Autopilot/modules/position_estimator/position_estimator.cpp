#include <modules/position_estimator/position_estimator.h>

Position_estimator::Position_estimator(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  kalman(n, m),
	  _time_sub(data_bus->time_node),
	  _modes_sub(data_bus->modes_node),
	  _imu_sub(data_bus->imu_node),
	  _baro_sub(data_bus->baro_node),
	  _gnss_sub(data_bus->gnss_node),
	  _of_sub(data_bus->of_node),
	  _ahrs_sub(data_bus->ahrs_node),
	  _telem_sub(data_bus->telem_node),
	  _local_pos_pub(data_bus->local_position_node)
{
}

void Position_estimator::update()
{
	_modes_data = _modes_sub.get();
	_time = _time_sub.get();
	_telem_data = _telem_sub.get();

	if (_modes_data.system_mode != System_mode::LOAD_PARAMS)
	{
		if (!_local_pos.converged)
		{
			update_initialization();
		}
		else
		{
			update_running();
		}
	}
}

void Position_estimator::update_initialization()
{
	_ahrs_data = _ahrs_sub.get();

	if (_gnss_sub.check_new())
	{
		_gnss_data = _gnss_sub.get();

		if (_gnss_data.fix)
		{
			_local_pos.ref_lat = _gnss_data.lat;
			_local_pos.ref_lon = _gnss_data.lon;
			_local_pos.ref_xy_set = true;
		}
	}

	if (_baro_sub.check_new())
	{
		_baro_data = _baro_sub.get();
		_local_pos.ref_alt = _baro_data.alt;
		_local_pos.ref_z_set = true;
	}

	if (_local_pos.ref_xy_set && _local_pos.ref_z_set)
	{
		_local_pos.converged = true;
	}
}

void Position_estimator::update_running()
{
	if (_imu_sub.check_new())
	{
		_imu_data = _imu_sub.get();

		if (_ahrs_sub.check_new())
		{
			_ahrs_data = _ahrs_sub.get();
			predict_accel();
		}

		if (_of_sub.check_new())
		{
			_of_data = _of_sub.get();

			if (is_of_reliable())
			{
				update_of_agl();
			}
		}
	}

	if (_gnss_sub.check_new())
	{
		_gnss_data = _gnss_sub.get();
		update_gps();
	}

	if (_baro_sub.check_new())
	{
		_baro_data = _baro_sub.get();
		update_baro();
	}
}

void Position_estimator::predict_accel()
{
	// Get IMU data
	Eigen::Vector3f acc_inertial(_imu_data.ax, _imu_data.ay, _imu_data.az);

	// Rotate inertial frame to NED
	Eigen::Vector3f acc_ned = inertial_to_ned(acc_inertial * G,
										  	  _ahrs_data.roll * DEG_TO_RAD,
											  _ahrs_data.pitch * DEG_TO_RAD,
											  _ahrs_data.yaw * DEG_TO_RAD);
	acc_ned(2) += G; // Gravity correction

	kalman.predict(acc_ned, get_a(_time.dt_s), get_b(_time.dt_s), get_q());

	update_plane();
}

void Position_estimator::update_gps()
{
	// Convert lat/lon to meters
	double gnss_north_meters, gnss_east_meters;
	lat_lon_to_meters(_local_pos.ref_lat, _local_pos.ref_lon,
					  _gnss_data.lat, _gnss_data.lon, &gnss_north_meters, &gnss_east_meters);

	Eigen::VectorXf y(2);
	y << gnss_north_meters,
		 gnss_east_meters;

	Eigen::MatrixXf H(2, n);
	H << 1, 0, 0, 0, 0, 0,
		 0, 1, 0, 0, 0, 0;

	Eigen::DiagonalMatrix<float, 2> R(param_get_float(EKF_GNSS_VAR), param_get_float(EKF_GNSS_VAR));

	kalman.update(R, H, y);

	update_plane();
}

void Position_estimator::update_baro()
{
	Eigen::VectorXf y(1);
	y << -(_baro_data.alt - _local_pos.ref_alt);

	Eigen::MatrixXf H(1, n);
	H << 0, 0, 1, 0, 0, 0;

	Eigen::DiagonalMatrix<float, 1> R(param_get_float(EKF_BARO_VAR));

	kalman.update(R, H, y);

	update_plane();
}

void Position_estimator::update_of_agl()
{
	float flow = sqrtf(powf(_of_data.x, 2) + powf(_of_data.y, 2));
	float angular_rate = sqrtf(powf(_imu_data.gx, 2) + powf(_imu_data.gy, 2)) * DEG_TO_RAD;
	float alt = _local_pos.gnd_spd / (flow - angular_rate);
	printf("OF AGL: %f\n", alt);
}

void Position_estimator::update_plane()
{
	Eigen::MatrixXf est = kalman.get_estimate();
	_local_pos.x = est(0, 0);
	_local_pos.y = est(1, 0);
	_local_pos.z = est(2, 0);
	_local_pos.vx = est(3, 0);
	_local_pos.vy = est(4, 0);
	_local_pos.vz = est(5, 0);
	_local_pos.gnd_spd = sqrtf(powf(est(3, 0), 2) + powf(est(4, 0), 2));
	_local_pos.terr_hgt = 0;
	_local_pos.timestamp = _hal->get_time_us();

	_local_pos_pub.publish(_local_pos);
}

// Function to rotate IMU measurements from inertial frame to NED frame
Eigen::Vector3f Position_estimator::inertial_to_ned(const Eigen::Vector3f& imu_measurement, float roll, float pitch, float yaw) {
    // Precompute trigonometric functions
    float cr = cosf(roll);  float sr = sinf(roll);
    float cp = cosf(pitch); float sp = sinf(pitch);
    float cy = cosf(yaw);   float sy = sinf(yaw);

    // Construct the rotation matrix (ZYX convention: yaw -> pitch -> roll)
    Eigen::Matrix3f R;
    R(0, 0) = cy * cp;
    R(0, 1) = cy * sp * sr - sy * cr;
    R(0, 2) = cy * sp * cr + sy * sr;
    R(1, 0) = sy * cp;
    R(1, 1) = sy * sp * sr + cy * cr;
    R(1, 2) = sy * sp * cr - cy * sr;
    R(2, 0) = -sp;
    R(2, 1) = cp * sr;
    R(2, 2) = cp * cr;

    // Rotate the measurement
    Eigen::Vector3f ned_measurement = R * imu_measurement;

    return ned_measurement;
}

bool Position_estimator::is_of_reliable()
{
	float flow = sqrtf(powf(_of_data.x, 2) + powf(_of_data.y, 2));
	return flow > param_get_int32(EKF_OF_MIN) && flow < param_get_int32(EKF_OF_MAX);
}

Eigen::MatrixXf Position_estimator::get_a(float dt)
{
	Eigen::MatrixXf A(n, n);
	A << 1, 0, 0, dt, 0, 0,
		 0, 1, 0, 0, dt, 0,
		 0, 0, 1, 0, 0, dt,
		 0, 0, 0, 1, 0, 0,
		 0, 0, 0, 0, 1, 0,
		 0, 0, 0, 0, 0, 1;

	return A;
}

Eigen::MatrixXf Position_estimator::get_b(float dt)
{
	Eigen::MatrixXf B(n, m);
	B << 0.5*dt*dt, 0, 0,
		 0, 0.5*dt*dt, 0,
		 0, 0, 0.5*dt*dt,
	     dt, 0, 0,
		 0, dt, 0,
		 0, 0, dt;

	return B;
}

Eigen::MatrixXf Position_estimator::get_q()
{
	Eigen::DiagonalMatrix<float, n> Q(1, 1, 1, 1, 1, 1);

	return Q;
}
