#include <ahrs.h>

/**
 * @brief Construct a new AHRS::AHRS object
 *
 * @param plane
 * @param hal
 */
AHRS::AHRS(Plane * plane, HAL * hal)
{
    _plane = plane;
    _hal = hal;
}

/**
 * @brief Setup AHRS
 */
void AHRS::setup()
{
    filter.begin(sample_frequency);
}

/**
 * @brief Check if new IMU data is available
 *
 * @return true
 * @return false
 */
bool AHRS::check_new_imu_data()
{
    return _plane->imu_timestamp != last_imu_timestamp;
}

/**
 * @brief Check if new compass data is available
 *
 * @return true
 * @return false
 */
bool AHRS::check_new_compass_data()
{
    return _plane->compass_timestamp != last_compass_timestamp;
}

/**
 * @brief Apply compass calibration
 *
 */
void AHRS::apply_compass_calibration()
{

}

/**
 * @brief Update AHRS
 */
void AHRS::update()
{
    time = _hal->get_time_us();
    prev_loop_time = time;

	if (check_new_imu_data()) {
		if (_plane->use_compass && check_new_compass_data()) {
			update_full();
		} else {
			update_imu();
		}
	}
}

/**
 * @brief Update filter with only IMU
 *
 */
void AHRS::update_imu()
{
    // Convert coordinate system from plane to Madgwick
    filter.updateIMU(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
                     _plane->imu_ax, _plane->imu_ay, _plane->imu_az);
    upload_results();
    last_imu_timestamp = _plane->imu_timestamp;
}

/**
 * @brief Update filter with both IMU and compass
 *
 */
void AHRS::update_full()
{
    filter.update(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
                  _plane->imu_ax, _plane->imu_ay, _plane->imu_az,
                  _plane->compass_mx, _plane->compass_my, _plane->compass_mz);
    upload_results();
    last_imu_timestamp = _plane->imu_timestamp;
    last_compass_timestamp = _plane->compass_timestamp;
}

/**
 * @brief Retrieve orientation from filter and insert it into plane struct
 *
 */
void AHRS::upload_results()
{
    // Convert coordinate system from Madgwick to plane
    _plane->ahrs_roll = filter.getRoll();
    _plane->ahrs_pitch = filter.getPitch();
    _plane->ahrs_yaw = filter.getYaw();
    _plane->ahrs_timestamp = time;
}
