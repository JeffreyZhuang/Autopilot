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

void rotateToWorldFrame(Eigen::Vector3f& accel_inertial,
                        Eigen::Quaternionf& imu_orientation,
                        Eigen::Vector3f& accel_world) {
    // Convert quaternion to rotation matrix
    Eigen::Matrix3f rotation_matrix = imu_orientation.toRotationMatrix();

    // Rotate accelerometer measurement
    accel_world = rotation_matrix * accel_inertial;
}

// Rotate inertial frame to ECF
void Navigation::read_imu()
{
//	Eigen::Vector3f acc_inertial(_plane->imu_ax * g, _plane->imu_ay * g, _plane->imu_az * g);
	acc_n = _plane->imu_ax * g;
	acc_e = _plane->imu_ay * g;
	last_imu_timestamp = _plane->imu_timestamp;
//
//	Eigen::Quaternionf ori(_plane->ahrs_q0, _plane->ahrs_q1, _plane->ahrs_q2, _plane->ahrs_q3); // w, x, y, z
//	Eigen::Vector3f acc_world;
//	rotateToWorldFrame(acc_inertial, ori, acc_world);
//
//	acc_n = acc_world(0);
//	acc_e = acc_world(1);
//	acc_d = acc_world(2) + g;
}

void Navigation::read_gnss()
{
	gnss_n = _plane->gnss_lat;
	gnss_e = _plane->gnss_lon;
	gnss_d = _plane->gnss_asl;
	last_gnss_timestamp = _plane->gnss_timestamp;
}

void Navigation:: read_ahrs()
{

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

bool Navigation::check_new_ahrs_data()
{
	return last_ahrs_timestamp != _plane->ahrs_timestamp;
}
