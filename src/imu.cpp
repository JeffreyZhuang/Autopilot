#include <hal_arduino.h>

/**
 * @brief Setup IMU
 * 
 */
void HAL_Arduino::setup_imu() {
    imu.setAccelODR(ICM42688::odr200);
    imu.setAccelFS(ICM42688::gpm4);

    imu.setGyroODR(ICM42688::odr200);
    imu.setGyroFS(ICM42688::dps500);

    imu.begin();
}

/**
 * @brief Poll data from IMU
 * 
 */
void HAL_Arduino::poll_imu() {
    if (imu.getAGT()) {
        // Rotate IMU to correct coordinate system
        _plane->imu_ax = -imu.accX();
        _plane->imu_ay = -imu.accY();
        _plane->imu_az = imu.accZ();
        _plane->imu_gx = -imu.gyrX();
        _plane->imu_gy = -imu.gyrY();
        _plane->imu_gz = imu.gyrZ();
        _plane->imu_temp = imu.temp();
        _plane->imu_timestamp = get_time_us();
    };
}