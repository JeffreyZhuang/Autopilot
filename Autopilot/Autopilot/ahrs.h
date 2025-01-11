#ifndef AHRS_H
#define AHRS_H

#include "madgwick.h"
#include "plane.h"
#include "hal.h"

enum class AHRS_state {
    INIT = 1,
    LIVE = 2
};

/**
 * @brief Attitude Heading Reference System
 *
 */
class AHRS
{
public:
    AHRS(Plane* plane, HAL* hal, float dt);
    void setup();
    void update();
private:
    void update_imu();
    void update_imu_mag();
    void upload_results();
    bool check_new_imu_data();
    bool check_new_compass_data();
    void apply_compass_calibration();

    Plane* _plane;
    HAL* _hal;

    AHRS_state ahrs_state = AHRS_state::INIT;

    Madgwick filter;
    float _dt = 0.01;
    uint64_t last_imu_timestamp;
    uint64_t last_compass_timestamp;
};

#endif
