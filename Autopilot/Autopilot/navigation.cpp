#include <navigation.h>

/**
 * @brief Construct a new Navigation:: Navigation object
 *
 * @param hal
 * @param plane
 */
Navigation::Navigation(HAL * hal, Plane * plane)
{
	hal = hal;
	_plane = plane;
}

/**
 * @brief Update navigation
 *
 */
void Navigation::execute()
{
	if (check_new_imu_data()) {
		prediction_step();
	}

	if (check_new_gnss_data())
	{
//		update_step();
	}
}

// Rotate inertial frame to ECF
void Navigation::read_imu()
{
	acc_n = _plane->imu_ax * g;
	acc_e = _plane->imu_ay * g;
	acc_d = _plane->imu_az * g;
	last_imu_timestamp = _plane->imu_timestamp;
}

void Navigation::read_gnss()
{
	gnss_n = _plane->gnss_lat;
	gnss_e = _plane->gnss_lon;
	gnss_d = _plane->gnss_asl;
	last_gnss_timestamp = _plane->gnss_timestamp;
}

void Navigation::prediction_step()
{
	read_imu();
	u << acc_n, acc_e;
	kalman.predict(u);

	Eigen::MatrixXf est = kalman.get_estimate();
	_plane->nav_pos_north = est(0, 0);
	_plane->nav_pos_east = est(1, 0);
	_plane->nav_vel_north = est(2, 0);
	_plane->nav_vel_east = est(3, 0);
}

void Navigation::update_step()
{
	read_gnss();
	y << gnss_n, gnss_e, 0, 0;
	kalman.update(H, y);

	Eigen::MatrixXf est = kalman.get_estimate();
	_plane->nav_pos_north = est(0, 0);
	_plane->nav_pos_east = est(1, 0);
	_plane->nav_vel_north = est(2, 0);
	_plane->nav_vel_east = est(3, 0);
}

/**
 * @brief Check if new IMU data is available
 *
 * @return true
 * @return false
 */
bool Navigation::check_new_imu_data()
{
    return last_imu_timestamp != _plane->imu_timestamp;
}

bool Navigation::check_new_gnss_data()
{
	return last_gnss_timestamp != _plane->gnss_timestamp;
}
