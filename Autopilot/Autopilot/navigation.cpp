#include <navigation.h>

/**
 * @brief Construct a new Navigation:: Navigation object
 *
 * @param hal
 * @param plane
 */
Navigation::Navigation(HAL * hal, Plane * plane) : kalman(n, m),
												   Q(1, 1, 1, 1),
												   R(1, 1),
												   H(1, 1, 0, 0)
{
	hal = hal;
	_plane = plane;

	// State vector: pos_n, pos_e, vel_n, vel_e
	// Input vector: acc_n, acc_e
	A << 1, 0, update_dt, 0,
		 0, 1, 0, 		  update_dt,
		 0, 0, 1, 		  0,
		 0, 0, 0, 	  	  1;

	B << 0.5*pow(predict_dt, 2), 0,
		 0, 					 0.5*pow(predict_dt, 2),
		 predict_dt, 			 0,
		 0, 					 predict_dt;

	kalman.set_matrices(A, B, Q, R);
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
	acc_n = _plane->imu_ax;
	acc_e = _plane->imu_ay;
	acc_d = _plane->imu_az;
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
}

void Navigation::update_step()
{
	read_gnss();
	y << gnss_n, gnss_e, 0, 0;
	kalman.update(H, y);

	_plane->nav_pos_north = kalman.get_estimate()(0, 1);
	_plane->nav_pos_east = kalman.get_estimate()(1, 1);
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
