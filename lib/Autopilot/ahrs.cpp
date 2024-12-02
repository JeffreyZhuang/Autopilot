#include <ahrs.h>

AHRS::AHRS(Plane * plane, HAL * hal) {
    _plane = plane;
    _hal = hal;
}

void AHRS::setup() {
    filter.begin(100);
}

void AHRS::update() {
    time = _hal->get_time_us();

    // Limit loop rate
    if (time - prev_loop_time > dt) {
        // Check and only run if there is new sensor data
        if (_plane->imu_timestamp != last_imu_timestamp) {
            // Check and only use compass if there is new compass data
            if (_plane->compass_timestamp != last_compass_timestamp) {
                update_full();
            } else {
                update_imu();
            }
        }

        prev_loop_time = time;
    }
}

void AHRS::update_imu() {
    filter.updateIMU(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz, 
                     _plane->imu_ax, _plane->imu_ay, _plane->imu_az);
    upload_results();
    last_imu_timestamp = _plane->imu_timestamp;
}

void AHRS::update_full() {
    filter.update(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz, 
                            _plane->imu_ax, _plane->imu_ay, _plane->imu_az, 
                            _plane->compass_mx, _plane->compass_my, _plane->compass_mz);
    upload_results();
    last_imu_timestamp = _plane->imu_timestamp;
    last_compass_timestamp = _plane->compass_timestamp;
}

void AHRS::upload_results() {
    _plane->ahrs_roll = filter.getRoll();
    _plane->ahrs_pitch = filter.getPitch();
    _plane->ahrs_yaw = filter.getYaw();
    _plane->ahrs_timestamp = time;
}