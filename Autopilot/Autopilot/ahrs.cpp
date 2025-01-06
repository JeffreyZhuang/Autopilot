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
	if (check_new_imu_data()) {
		// Compass not calibrated yet so for testing purposes
//		if (_plane->use_compass && check_new_compass_data()) {
		if (0) {
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
    filter.updateIMU(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
                     -_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az);
    last_imu_timestamp = _plane->imu_timestamp;

    upload_results();
}

/**
 * @brief Update filter with both IMU and compass
 *
 */
void AHRS::update_full()
{
    filter.update(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz,
                  -_plane->imu_ax, -_plane->imu_ay, -_plane->imu_az,
                  -_plane->compass_mx, -_plane->compass_my, -_plane->compass_mz);
    last_imu_timestamp = _plane->imu_timestamp;
    last_compass_timestamp = _plane->compass_timestamp;

    upload_results();
}

/**
 * @brief Retrieve orientation from filter and insert it into plane struct
 *
 */
void AHRS::upload_results()
{
	// Madgwick library coordinate system is inverted, so rotate it back to the correct system
    _plane->ahrs_roll = filter.getRoll();
    _plane->ahrs_pitch = filter.getPitch();
    _plane->ahrs_yaw = filter.getYaw();
    _plane->ahrs_q0 = filter.get_q0();
    _plane->ahrs_q1 = filter.get_q1();
    _plane->ahrs_q2 = filter.get_q2();
    _plane->ahrs_q3 = filter.get_q3();
    _plane->ahrs_timestamp = _plane->time;
}
