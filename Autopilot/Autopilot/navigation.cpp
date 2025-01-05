#include <navigation.h>

/**
 * @brief Construct a new Navigation:: Navigation object
 *
 * @param hal
 * @param plane
 */
Navigation::Navigation(HAL* hal, Plane* plane) : kalman(n, m, get_a(), get_b(), get_q(), get_r())
{
	_hal = hal;
	_plane = plane;
}

Eigen::MatrixXf Navigation::get_a()
{
	float predict_dt = 0.01;
	Eigen::MatrixXf A(n, n);
	A << 1, 0, 0, predict_dt, 0, 0,
		 0, 1, 0, 0, predict_dt, 0,
		 0, 0, 1, 0, 0, predict_dt,
		 0, 0, 0, 1, 0, 0,
		 0, 0, 0, 0, 1, 0,
		 0, 0, 0, 0, 0, 1;
	return A;
}

Eigen::MatrixXf Navigation::get_b()
{
	float predict_dt = 0.01;
	Eigen::MatrixXf B(n, m);
	B << 0.5*predict_dt*predict_dt, 0, 0,
			  0, 0.5*predict_dt*predict_dt, 0,
			  0, 0, 0.5*predict_dt*predict_dt,
			  predict_dt, 0, 0,
			  0, predict_dt, 0,
			  0, 0, predict_dt;
	return B;
}

Eigen::MatrixXf Navigation::get_q()
{
	Eigen::DiagonalMatrix<float, n> Q(1, 1, 1, 1, 1, 1);
	return Q;
}

Eigen::MatrixXf Navigation::get_r()
{
	Eigen::DiagonalMatrix<float, m> R(1, 1, 1);
	return R;
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

//	if (check_new_baro_data())
//	{
//		update_baro();
//	}
}

void Navigation::predict_imu()
{
	read_imu();

	Eigen::VectorXf u(m);
	u << acc_n,
		 acc_e,
		 acc_d;

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

	Eigen::VectorXf y(n);
	y << gnss_n,
		 gnss_e,
		 0,
		 0,
		 0,
		 0;

	Eigen::DiagonalMatrix<float, n> H(1, 1, 0, 0, 0, 0);

	kalman.update(H, y);
	update_plane();
}

void Navigation::update_baro()
{
	float baro = 0;

	Eigen::VectorXf y(n);
	y << 0, 0, baro, 0, 0, 0;

	Eigen::DiagonalMatrix<float, n> H(0, 0, 1, 0, 0, 0);

	kalman.update(H, y);
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
}

// Rotate inertial frame to ECF
void Navigation::read_imu()
{
	Eigen::Vector3f acc_inertial(_plane->imu_ax, _plane->imu_ay, _plane->imu_az);
	last_imu_timestamp = _plane->imu_timestamp;

	Eigen::Quaternionf q(_plane->ahrs_q0, _plane->ahrs_q1, _plane->ahrs_q2, _plane->ahrs_q3);
	Eigen::Vector3f acc_world = q * acc_inertial;

	acc_n = acc_world(0) * g;
	acc_e = acc_world(1) * g;
	acc_d = (acc_world(2) + 1) * g;
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
