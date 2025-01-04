#include <navigation.h>

/**
 * @brief Construct a new Navigation:: Navigation object
 *
 * @param hal
 * @param plane
 */
Navigation::Navigation(HAL * hal, Plane * plane)
{
	_hal = hal;
	_plane = plane;
}

/**
 * @brief Update navigation
 *
 */
void Navigation::execute()
{
	if (check_new_imu_data()) {
		predict_imu();
	}

	if (check_new_gnss_data())
	{
//		update_step();
	}
}

void Navigation::predict_imu()
{
	read_imu();

	Eigen::MatrixXf u(3, 1);
	u << acc_n, acc_e, acc_d;

	kalman.predict(u);

	update_plane();

//	if (fabs(_plane->nav_pos_north) > 0.5)
//	{
//		kalman.reset();
//	}
}

void Navigation::update_gps()
{
	read_gnss();

	Eigen::MatrixXf y(6, 1);
	y << gnss_n, gnss_e, 0, 0, 0, 0;

	Eigen::MatrixXf H(6, 6);
	H << 1, 0, 0, 0, 0, 0,
		 0, 1, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0;

	kalman.update(H, y);
	update_plane();
}

void Navigation::update_baro()
{

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
}

Eigen::Vector3f rotateInertialToWorld(const Eigen::Vector3f& v_inertial, float phi, float theta, float psi) {
    // Compute trigonometric values
    float c_phi = cos(phi), s_phi = sin(phi);
    float c_theta = cos(theta), s_theta = sin(theta);
    float c_psi = cos(psi), s_psi = sin(psi);

    // Roll rotation matrix (R_x)
    Eigen::Matrix3f R_x;
    R_x << 1, 0, 0,
           0, c_phi, -s_phi,
           0, s_phi, c_phi;

    // Pitch rotation matrix (R_y)
    Eigen::Matrix3f R_y;
    R_y << c_theta, 0, s_theta,
           0, 1, 0,
           -s_theta, 0, c_theta;

    // Yaw rotation matrix (R_z)
    Eigen::Matrix3f R_z;
    R_z << c_psi, -s_psi, 0,
           s_psi, c_psi, 0,
           0, 0, 1;

    // Composite rotation matrix
    Eigen::Matrix3f R_world_inertial = R_z * R_y * R_x;

    // Transform the vector
    Eigen::Vector3f v_world = R_world_inertial * v_inertial;

    return v_world;
}

// Rotate inertial frame to ECF
void Navigation::read_imu()
{
	Eigen::Vector3f acc_inertial(_plane->imu_ax * g, _plane->imu_ay * g, _plane->imu_az * g);
	last_imu_timestamp = _plane->imu_timestamp;

	Eigen::Vector3f acc_world = rotateInertialToWorld(acc_inertial,
													  _plane->ahrs_roll * M_PI / 180,
													  _plane->ahrs_pitch * M_PI / 180,
													  (_plane->ahrs_yaw - 180.0f) * M_PI / 180);

	acc_n = acc_world(0);
	acc_e = acc_world(1);
	acc_d = acc_world(2) + g;
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
