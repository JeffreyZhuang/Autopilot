#include <navigation.h>

/**
 * @brief Construct a new Navigation:: Navigation object
 *
 * @param hal
 * @param plane
 */
Navigation::Navigation(HAL* hal, Plane* plane) : kalman(n, m, get_a(predict_dt), get_b(predict_dt), get_q())
{
	_hal = hal;
	_plane = plane;
}

Eigen::MatrixXf Navigation::get_a(float dt)
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

Eigen::MatrixXf Navigation::get_b(float dt)
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

Eigen::MatrixXf Navigation::get_q()
{
	Eigen::DiagonalMatrix<float, n> Q(1, 1, 1, 1, 1, 1);
	return Q;
}

/**
 * @brief Update navigation
 *
 */
void Navigation::execute()
{
	if (navigationState == NavigationState::INITIALIZATION)
	{
		execute_initialization();
	}
	else if (navigationState == NavigationState::LIVE)
	{
		execute_live();
	}
}

// Calibrate sensors
// Easier if I move this to the drivers...
void Navigation::execute_initialization()
{
	if (check_new_baro_data())
	{
		_plane->baro_offset = _plane->baro_alt;
	}

	printf("Sats: %d\n", _plane->gnss_sats);

	// Check if GNSS is locked
	if (check_new_gnss_data() && _plane->gnss_sats > 5 && _plane->gnss_lat > 0)
	{
		// Set center GPS coordinates
		_plane->gnss_center_lat = _plane->gnss_lat;
		_plane->gnss_center_lon = _plane->gnss_lon;

		printf("Center: %f %f\n", _plane->gnss_center_lat, _plane->gnss_center_lon);

		navigationState = NavigationState::LIVE;
	}
}

void Navigation::execute_live()
{
	if (check_new_imu_data()) {
		predict_imu();
	}

	if (check_new_gnss_data())
	{
//		update_gps();
	}

	if (check_new_baro_data())
	{
//		uint64_t start = _hal->get_time_us();
		update_baro();
//		printf("%ld ", (uint32_t)(_hal->get_time_us() - start));
	}
}

void Navigation::predict_imu()
{
	read_imu();

	Eigen::VectorXf u(m);
	u << _plane->nav_acc_north,
		 _plane->nav_acc_east,
		 _plane->nav_acc_down;

	kalman.predict(u);

	update_plane();
}

void Navigation::update_gps()
{
	read_gnss();

	Eigen::VectorXf y(2);
	y << gnss_n,
		 gnss_e;

	Eigen::MatrixXf H(2, n);
	H << 1, 0, 0, 0, 0, 0,
		 0, 1, 0, 0, 0, 0;

	Eigen::DiagonalMatrix<float, 2> R(1, 1);

	kalman.update(R, H, y);

	update_plane();
}

void Navigation::update_baro()
{
	Eigen::VectorXf y(1);
	y << -(_plane->baro_alt - _plane->baro_offset); // Multiply by -1 to put in correct coordinate system
	last_baro_timestamp = _plane->baro_timestamp;

	Eigen::MatrixXf H(1, n);
	H << 0, 0, 1, 0, 0, 0;

	Eigen::DiagonalMatrix<float, 1> R(10000);

	kalman.update(R, H, y);

	update_plane();
}

void Navigation::update_plane()
{
	Eigen::MatrixXf est = kalman.get_estimate();
	_plane->nav_pos_north = est(0, 0);
	_plane->nav_pos_east = est(1, 0);
	_plane->nav_pos_down = est(2, 0);
	_plane->nav_vel_north = est(3, 0);
	_plane->nav_vel_east = est(4, 0);
	_plane->nav_vel_down = est(5, 0);
	_plane->nav_timestamp = _hal->get_time_us();
}

// Rotate inertial frame to ECF
void Navigation::read_imu()
{
	// Get IMU data
	Eigen::Vector3f acc_inertial(_plane->imu_ax, _plane->imu_ay, _plane->imu_az);
	last_imu_timestamp = _plane->imu_timestamp;

	// Get quaternion from AHRS
	Eigen::Quaternionf q(_plane->ahrs_q0, _plane->ahrs_q1, _plane->ahrs_q2, _plane->ahrs_q3);

	// Convert from inertial frame to ECF
	Eigen::Vector3f acc_world = q * acc_inertial * g;
	acc_world(2) += g; // Gravity correction

	_plane->nav_acc_north = acc_world(0);
	_plane->nav_acc_east = acc_world(1);
	_plane->nav_acc_down = acc_world(2);
}

void Navigation::read_gnss()
{
	// Convert from lat/lon to meters
	gnss_n = _plane->gnss_lat;
	gnss_e = _plane->gnss_lon;
	gnss_d = _plane->gnss_asl;
	last_gnss_timestamp = _plane->gnss_timestamp;
}

bool Navigation::check_new_imu_data()
{
    return last_imu_timestamp != _plane->imu_timestamp;
}

bool Navigation::check_new_gnss_data()
{
	return last_gnss_timestamp != _plane->gnss_timestamp;
}

bool Navigation::check_new_baro_data()
{
	return last_baro_timestamp != _plane->baro_timestamp;
}

