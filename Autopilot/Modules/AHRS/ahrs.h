#ifndef AHRS_H
#define AHRS_H

#include "Lib/IMU_EKF/ESKF.h"
#include "plane.h"
#include "hal.h"
#include <stdio.h>

/**
 * @brief Attitude Heading Reference System
 *
 */
class AHRS
{
public:
    AHRS(HAL* hal, Plane* plane, float dt);
    void setup();
    bool set_initial_state();
    void update();
    bool initial_state_set = false;
private:
    void update_imu();
    void update_imu_mag();
    void publish_ahrs();
    bool check_new_imu_data();
    bool check_new_compass_data();
    void apply_compass_calibration();

    Plane* _plane;
    HAL* _hal;
//    Madgwick filter;
    IMU_EKF::ESKF<float> filter;
    Eigen::Matrix<float, 3, 3> W; // soft-iron
    Eigen::Matrix<float, 3, 1> V; // hard-iron
    float incl = 0; // inclination
    float B; // geomagnetic field strength

    float _dt;
    uint64_t last_imu_timestamp = 0;
    uint64_t last_compass_timestamp = 0;
};

#endif
