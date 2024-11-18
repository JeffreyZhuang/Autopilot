#include <ahrs.h>

AHRS::AHRS(Vehicle * vehicle) {
    _vehicle = vehicle;
}

void AHRS::setup() {
    filter.begin(200);
}

void AHRS::update() {
    filter.update(_vehicle->imu_gx, _vehicle->imu_gy, _vehicle->imu_gz, 
                  _vehicle->imu_ax, _vehicle->imu_ay, _vehicle->imu_az, 
                  _vehicle->compass_mx, _vehicle->compass_my, _vehicle->compass_mz);
    _vehicle->ahrs_roll = filter.getRoll();
    _vehicle->ahrs_pitch = filter.getPitch();
    _vehicle->ahrs_yaw = filter.getYaw();
}