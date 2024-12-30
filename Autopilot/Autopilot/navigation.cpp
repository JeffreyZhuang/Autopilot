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
		last_imu_timestamp = _plane->imu_timestamp;
	}

	if (check_new_gnss_data())
	{
		update_step();
		last_gnss_timestamp = _plane->gnss_timestamp;
	}
}

void Navigation::prediction_step()
{

}

void Navigation::update_step()
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
