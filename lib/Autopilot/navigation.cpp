#include <navigation.h>

Navigation::Navigation(HAL * hal, Plane * plane) {
    _hal = hal;
    _plane = plane;
}

void Navigation::update() {
    uint32_t time = _hal->get_time_us();

    // Limit loop rate
    if (time - prev_loop_time > dt) {
        if (last_imu_timestamp != _plane->imu_timestamp) {
            update_accelerometer();
        }

        prev_loop_time = time;
    }
}

void Navigation::update_accelerometer() {
    last_imu_timestamp = _plane->imu_timestamp;
}