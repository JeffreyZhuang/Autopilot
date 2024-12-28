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
    uint32_t time = _hal->get_time_us();

    // Limit loop rate
    if (time - prev_loop_time > dt) {
        if (check_new_imu_data()) {
            update_accelerometer();
        }

        prev_loop_time = time;
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
