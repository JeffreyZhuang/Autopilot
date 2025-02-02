#ifndef AHRS_H
#define AHRS_H

#include "madgwick.h"
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
    void set_state();
    void update();
private:
    void update_imu();
    void update_imu_mag();
    void publish_ahrs();
    bool check_new_imu_data();
    bool check_new_compass_data();
    void apply_compass_calibration();

    Plane* _plane;
    HAL* _hal;
    Madgwick filter;

    float _dt;
    uint64_t last_imu_timestamp;
    uint64_t last_compass_timestamp;
};

#endif
