#ifndef AHRS_H
#define AHRS_H

#include <madgwick.h>
#include <plane.h>
#include <hal.h>

class AHRS
{
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
    void apply_compass_calibration();

    // Compass calibration data
    const float hard_iron[3] = {52.67, -5.27, 81.54};
    const float soft_iron[3][3] = {{1.031, 0.015, -0.0032},
                                   {0.015, 0.967, -0.025},
                                   {-0.032, -0.025, 1.005}};
    const float mag_decl = 0.0;

    Plane * _plane;
    HAL * _hal;
    Madgwick filter;
    uint32_t last_imu_timestamp;
    uint32_t last_compass_timestamp;
    uint32_t prev_loop_time;
    uint32_t dt = 10000; // us
    uint64_t time;
};

#endif
