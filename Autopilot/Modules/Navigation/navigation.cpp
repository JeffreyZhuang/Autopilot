#include <navigation.h>

// Probably kalman filter blacking out because GNSS returns NaN and that gets fed into kalman

/**
 * @brief Construct a new Navigation:: Navigation object
 *
 * @param hal
 * @param plane
 */
Navigation::Navigation(HAL* hal, Plane* plane, float predict_dt) : kalman(n, m, get_a(predict_dt), get_b(predict_dt), get_q())
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

bool Navigation::check_gnss_lock()
{
	return (_plane->gnss_sats > 5) && (fabs(_plane->gnss_lat) > 0) && (fabs(_plane->gnss_lat) > 0);
}

// Calibrate sensors
// Easier if I move this to the drivers... or maybe not because waiting for converging
// Wait for converging especially AHRS heading
void Navigation::execute_initialization()
{
	// Do this in autopilot instead of navigation?
	if (check_new_baro_data())
	{
		_plane->baro_offset = _plane->baro_alt;
	}

	if (check_new_gnss_data())
	{
		if (check_gnss_lock())
		{
			// Use the first GPS fix as the center
			_plane->gnss_center_lat = _plane->gnss_lat;
			_plane->gnss_center_lon = _plane->gnss_lon;

			navigationState = NavigationState::LIVE;
		}
	}
}

void Navigation::execute_live()
{
	if (check_new_imu_data()) {
		predict_imu();
	}

	if (check_new_gnss_data())
	{
		update_gps();
	}

	if (check_new_baro_data())
	{
		update_baro();
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

//	printf("%.2f %.2f\n", gnss_n, gnss_e);

	Eigen::VectorXf y(2);
	y << gnss_n,
		 gnss_e;

	Eigen::MatrixXf H(2, n);
	H << 1, 0, 0, 0, 0, 0,
		 0, 1, 0, 0, 0, 0;

	Eigen::DiagonalMatrix<float, 2> R(1000000, 1000000);

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
	_plane->nav_airspeed = sqrtf(powf(_plane->nav_vel_north, 2) + powf(_plane->nav_vel_east, 2));
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
	// Use double instead of float because float not enough...
	// Tie down the modules together on wood

	// Convert from lat/lon to meters
	double earth_radius = 6371000;
	double lat1 = _plane->gnss_center_lat * M_PI / 180.0;
	double lon1 = _plane->gnss_center_lon * M_PI / 180.0;
	double lat2 = _plane->gnss_lat * M_PI / 180.0;
	double lon2 = _plane->gnss_lon * M_PI / 180.0;
	last_gnss_timestamp = _plane->gnss_timestamp;

	gnss_e = (lon2 - lon1) * earth_radius * cos((lat1 + lat2) / 2);
	gnss_n = (lat2 - lat1) * earth_radius;
	gnss_d = _plane->gnss_asl;
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

