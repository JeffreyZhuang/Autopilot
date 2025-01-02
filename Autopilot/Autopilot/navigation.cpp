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

    // Use identity matrix as initial covariance matrix
    const float Pdiag[EKF_N] = {1, 1, 1, 1, 1, 1};

    ekf_initialize(&ekf, Pdiag);
}

/**
 * @brief Update navigation
 *
 */
void Navigation::execute()
{
    time = _hal->get_time_us();

	if (check_new_imu_data()) {
		prediction_step();
	}

	if (check_new_gnss_data())
	{
		update_step();
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

	// Observation vector
	const float z[EKF_M] = {acc_n, acc_e, acc_d};

	ekf_predict(&ekf, ekf.x, F, Q);
}

void Navigation::update_step()
{
	read_gnss();

	const float hx[EKF_M] = {};
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
