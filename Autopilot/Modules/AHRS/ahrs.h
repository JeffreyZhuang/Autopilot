#ifndef AHRS_H
#define AHRS_H

#include "plane.h"
#include "hal.h"
#include <stdio.h>
#include <math.h>
#include "Lib/q-mekf/q_mekf.h"

//extern "C"
//{
//#include "Lib/KalmanQuatAtt/KalmanQuatAtt.h"
//}

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

    Eigen::Vector3f sigma_a = {20.78e-3, 20.78e-3, 20.78e-3};
	Eigen::Vector3f sigma_g = {100*M_PI/180, 100*M_PI/180, 100*M_PI/180};
	Eigen::Vector3f sigma_m = {3.2e-3, 3.2e-3, 4.1e-3};
	QuaternionMEKF<float, true> mekf;

    float _dt;
    uint64_t last_imu_timestamp = 0;
    uint64_t last_compass_timestamp = 0;
};

#endif
