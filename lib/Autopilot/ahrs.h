#ifndef AHRS_H
#define AHRS_H

#include <MadgwickAHRS.h>
#include <plane.h>
#include <hal.h>

/**
 * @brief Attitude Heading Reference System
 * 
 */
class AHRS {
public:
    AHRS(Plane * plane, HAL * hal);

    void setup();
    void update();
private:
    void update_imu();
    void update_full();
    void upload_results();
    bool check_new_imu_data();
    bool check_new_compass_data();

    Plane * _plane;
    HAL * _hal;
    Madgwick filter;
    uint32_t last_imu_timestamp;
    uint32_t last_compass_timestamp;
    uint32_t prev_loop_time;
    uint32_t dt = 10000; // 100hz
    uint32_t time;
};

#endif