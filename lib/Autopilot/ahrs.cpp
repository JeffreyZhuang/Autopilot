#include <ahrs.h>

AHRS::AHRS(Plane * plane, HAL * hal) {
    _plane = plane;
    _hal = hal;
}

void AHRS::setup() {
    filter.begin(200);
}

void AHRS::update() {
    // Limit loop rate to 100Hz
    if (_hal->get_time_us() - prev_loop_time > 10000) {
        // Check and only run if there is new sensor data
        if (_plane->imu_timestamp != last_imu_timestamp) {
            // Check and only use compass if there is new compass data
            if (_plane->compass_timestamp != last_compass_timestamp) {
                update_full();
            } else {
                update_imu();
            }
        }

        prev_loop_time = _hal->get_time_us();
    }
}

void AHRS::update_imu() {
    filter.updateIMU(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz, 
                     _plane->imu_ax, _plane->imu_ay, _plane->imu_az);
    _plane->ahrs_roll = filter.getRoll();
    _plane->ahrs_pitch = filter.getPitch();
    _plane->ahrs_yaw = filter.getYaw();
    last_imu_timestamp = _plane->imu_timestamp;
}

void AHRS::update_full() {
    filter.update(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz, 
                            _plane->imu_ax, _plane->imu_ay, _plane->imu_az, 
                            _plane->compass_mx, _plane->compass_my, _plane->compass_mz);
    _plane->ahrs_roll = filter.getRoll();
    _plane->ahrs_pitch = filter.getPitch();
    _plane->ahrs_yaw = filter.getYaw();
    last_compass_timestamp = _plane->compass_timestamp;
}