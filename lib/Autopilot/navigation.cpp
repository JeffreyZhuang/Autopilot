#include <navigation.h>

Navigation::Navigation(Plane * plane) {
    _plane = plane;
}

void Navigation::update() {
    if (last_imu_timestamp != _plane->imu_timestamp) {
        update_accelerometer();
    }
}

void Navigation::update_accelerometer() {
    last_imu_timestamp = _plane->imu_timestamp;
}