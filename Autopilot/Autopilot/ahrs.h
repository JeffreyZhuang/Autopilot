#ifndef AHRS_H
#define AHRS_H

#include <madgwick.h>
#include <plane.h>
#include <hal.h>

/**
 * @brief Attitude Heading Reference System
 *
 */
class AHRS
{
public:
    AHRS(Plane* plane, HAL* hal);

    void setup();
    void update();
private:
    void update_imu();
    void update_full();
    void upload_results();
    bool check_new_imu_data();
    bool check_new_compass_data();
    void apply_compass_calibration();

    Plane* _plane;
    HAL* _hal;

    Madgwick filter;
    float sample_frequency = 100;
    uint32_t last_imu_timestamp;
    uint32_t last_compass_timestamp;
    uint64_t time;
    uint64_t prev_time;
};

#endif
