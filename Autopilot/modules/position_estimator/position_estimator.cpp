#include <modules/position_estimator/position_estimator.h>

Position_estimator::Position_estimator(HAL* hal, Plane* plane)
	: Module(hal, plane),
	  kalman(n, m),
	  avg_baro(window_len, window_baro),
	  avg_lat(window_len, window_lat),
	  avg_lon(window_len, window_lon)
{
}

void Position_estimator::update()
{
	if (_plane->system_mode != System_mode::CONFIG)
	{
		switch (pos_estimator_state)
		{
		case Pos_estimator_state::INITIALIZATION:
			update_initialization();
			break;
		case Pos_estimator_state::RUNNING:
			update_running();
			break;
		}
	}
}

void Position_estimator::update_initialization()
{
	if (check_new_baro_data())
	{
		avg_baro.add(_plane->baro_alt);
	}

	if (check_new_baro_data() &&
		check_new_gnss_data() &&
		avg_baro.getFilled() &&
		_plane->ahrs_converged)
	{
		// Set barometer home position
		_plane->baro_offset = avg_baro.getAverage();

		pos_estimator_state = Pos_estimator_state::RUNNING;
	}
}

void Position_estimator::update_running()
{
	if (check_new_imu_data())
	{
		if (check_new_ahrs_data())
		{
			last_ahrs_timestamp = _plane->ahrs_timestamp;
			last_imu_timestamp = _plane->imu_timestamp;
			predict_imu();
		}

		if (check_new_of_data() && is_of_reliable())
		{
			last_imu_timestamp = _plane->imu_timestamp;
			last_of_timestamp = _plane->of_timestamp;
			update_of_agl();
		}
	}

	if (check_new_gnss_data())
	{
		last_gnss_timestamp = _plane->gnss_timestamp;
		update_gps();
	}

	if (check_new_baro_data())
	{
		last_baro_timestamp = _plane->baro_timestamp;
		update_baro();
	}
}

void Position_estimator::predict_imu()
{
	// Get IMU data
	Eigen::Vector3f acc_inertial(_plane->imu_ax, _plane->imu_ay, _plane->imu_az);

	// Rotate inertial frame to NED
	Eigen::Vector3f acc_ned = inertial_to_ned(acc_inertial * G,
										  _plane->ahrs_roll * DEG_TO_RAD,
										  _plane->ahrs_pitch * DEG_TO_RAD,
										  _plane->ahrs_yaw * DEG_TO_RAD);
	acc_ned(2) += G; // Gravity correction

	_plane->nav_acc_north = acc_ned(0);
	_plane->nav_acc_east = acc_ned(1);
	_plane->nav_acc_down = acc_ned(2);

	kalman.predict(acc_ned, get_a(_plane->dt_s), get_b(_plane->dt_s), get_q());

	update_plane();
}

void Position_estimator::update_gps()
{
	// Convert lat/lon to meters
	double gnss_north_meters, gnss_east_meters;
	lat_lon_to_meters(_plane->home_lat,
					  _plane->home_lon,
					  _plane->gnss_lat,
					  _plane->gnss_lon,
					  &gnss_north_meters,
					  &gnss_east_meters);

	Eigen::VectorXf y(3);
	y << gnss_north_meters,
		 gnss_east_meters,
		 -_plane->gnss_asl; // Need to subtract offset!

	Eigen::MatrixXf H(3, n);
	H << 1, 0, 0, 0, 0, 0,
		 0, 1, 0, 0, 0, 0,
		 0, 0, 1, 0, 0, 0;

	Eigen::DiagonalMatrix<float, 3> R(get_params()->pos_estimator.gnss_r,
									  get_params()->pos_estimator.gnss_r,
									  get_params()->pos_estimator.gnss_alt_r);

	kalman.update(R, H, y);

	update_plane();
}

void Position_estimator::update_baro()
{
	Eigen::VectorXf y(1);
	y << -(_plane->baro_alt - _plane->baro_offset);

	Eigen::MatrixXf H(1, n);
	H << 0, 0, 1, 0, 0, 0;

	Eigen::DiagonalMatrix<float, 1> R(get_params()->pos_estimator.baro_r);

	kalman.update(R, H, y);

	update_plane();
}

void Position_estimator::update_of_agl()
{
	float flow = sqrtf(powf(_plane->of_x, 2) + powf(_plane->of_y, 2));
	float angular_rate = sqrtf(powf(_plane->imu_gx, 2) + powf(_plane->imu_gy, 2)) * DEG_TO_RAD;
	float alt = _plane->nav_gnd_spd / (flow - angular_rate);
	printf("OF AGL: %f\n", alt);
}

void Position_estimator::update_plane()
{
	Eigen::MatrixXf est = kalman.get_estimate();
	_plane->nav_pos_north = est(0, 0);
	_plane->nav_pos_east = est(1, 0);
	_plane->nav_pos_down = est(2, 0);
	_plane->nav_vel_north = est(3, 0);
	_plane->nav_vel_east = est(4, 0);
	_plane->nav_vel_down = est(5, 0);
	_plane->nav_gnd_spd = sqrtf(powf(_plane->nav_vel_north, 2) + powf(_plane->nav_vel_east, 2));
	_plane->nav_timestamp = _hal->get_time_us();
	_plane->nav_converged = pos_estimator_state == Pos_estimator_state::RUNNING;
}

bool Position_estimator::check_new_imu_data()
{
    return _plane->imu_timestamp > last_imu_timestamp;
}

bool Position_estimator::check_new_gnss_data()
{
	return _plane->gnss_timestamp > last_gnss_timestamp;
}

bool Position_estimator::check_new_baro_data()
{
	return _plane->baro_timestamp > last_baro_timestamp;
}

bool Position_estimator::check_new_ahrs_data()
{
	return _plane->ahrs_timestamp > last_ahrs_timestamp;
}

bool Position_estimator::check_new_of_data()
{
	return _plane->of_timestamp > last_of_timestamp;
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
	float flow = sqrtf(powf(_plane->of_x, 2) + powf(_plane->of_y, 2));
	return flow > get_params()->sensors.of_min && flow < get_params()->sensors.of_max;
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
