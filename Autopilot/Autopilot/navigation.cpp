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
void Navigation::update()
{
    uint64_t time = _hal->get_time_us();
    prev_loop_time = time;

	if (check_new_imu_data()) {
		update_accelerometer();
	}
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

/**
 * @brief Update filter with only accelerometer
 *
 */
void Navigation::update_accelerometer()
{
    last_imu_timestamp = _plane->imu_timestamp;
}
