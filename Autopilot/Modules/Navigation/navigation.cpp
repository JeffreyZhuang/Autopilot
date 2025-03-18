#include "navigation.h"

Navigation::Navigation(HAL* hal, Plane* plane)
	: kalman(n, m, get_a(hal->get_main_dt()), get_b(hal->get_main_dt()), get_q()),
	  avg_baro(window_len, window_baro),
	  avg_lat(window_len, window_lat),
	  avg_lon(window_len, window_lon)
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

void Navigation::update_initialization()
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

		nav_state = Nav_state::RUNNING;
	}
}

/**
 * @brief Update navigation
 *
 */
void Navigation::update()
{
	if (_plane->system_mode != System_mode::CONFIG)
	{
		switch (nav_state)
		{
		case Nav_state::INITIALIZATION:
			update_initialization();
			break;
		case Nav_state::RUNNING:
			update_running();
			break;
		}
	}
}

void Navigation::update_running()
{
	if (check_new_ahrs_data() && check_new_imu_data())
	{
		last_ahrs_timestamp = _plane->ahrs_timestamp;
		last_imu_timestamp = _plane->imu_timestamp;
		predict_imu();
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

void Navigation::predict_imu()
{
	// Get IMU data
	Eigen::Vector3f acc_inertial(_plane->imu_ax, _plane->imu_ay, _plane->imu_az);

	// Rotate inertial frame to NED
	Eigen::Vector3f acc_ned = inertial_to_ned(acc_inertial * G,
										  _plane->ahrs_roll * DEG_TO_RAD,
										  _plane->ahrs_pitch * DEG_TO_RAD,
										  _plane->ahrs_yaw * DEG_TO_RAD);
	acc_ned(2) += g; // Gravity correction

	_plane->nav_acc_north = acc_ned(0);
	_plane->nav_acc_east = acc_ned(1);
	_plane->nav_acc_down = acc_ned(2);

	kalman.predict(acc_ned);

	update_plane();
}

void Navigation::update_gps()
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

	Eigen::DiagonalMatrix<float, 3> R(get_params()->gnss_r, get_params()->gnss_r, get_params()->gnss_alt_r);

	kalman.update(R, H, y);

	update_plane();
}

void Navigation::update_baro()
{
	Eigen::VectorXf y(1);
	y << -(_plane->baro_alt - _plane->baro_offset); // Multiply by -1 to put in correct coordinate system

	Eigen::MatrixXf H(1, n);
	H << 0, 0, 1, 0, 0, 0;

	Eigen::DiagonalMatrix<float, 1> R(get_params()->baro_r);

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
	_plane->nav_converged = nav_state == Nav_state::RUNNING;
}

bool Navigation::check_new_imu_data()
{
    return _plane->imu_timestamp > last_imu_timestamp;
}

bool Navigation::check_new_gnss_data()
{
	return _plane->gnss_timestamp > last_gnss_timestamp;
}

bool Navigation::check_new_baro_data()
{
	return _plane->baro_timestamp > last_baro_timestamp;
}

bool Navigation::check_new_ahrs_data()
{
	return _plane->ahrs_timestamp > last_ahrs_timestamp;
}

// Function to rotate IMU measurements from inertial frame to NED frame
Eigen::Vector3f Navigation::inertial_to_ned(const Eigen::Vector3f& imu_measurement, float roll, float pitch, float yaw) {
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
