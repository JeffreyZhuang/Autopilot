#include <ahrs.h>

AHRS::AHRS(Plane * plane) {
    _plane = plane;
}

void AHRS::setup() {
    filter.begin(200);
}

void AHRS::update() {
    filter.update(_plane->imu_gx, _plane->imu_gy, _plane->imu_gz, 
                  _plane->imu_ax, _plane->imu_ay, _plane->imu_az, 
                  _plane->compass_mx, _plane->compass_my, _plane->compass_mz);
    _plane->ahrs_roll = filter.getRoll();
    _plane->ahrs_pitch = filter.getPitch();
    _plane->ahrs_yaw = filter.getYaw();
}