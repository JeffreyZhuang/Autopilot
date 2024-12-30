#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <plane.h>
#include <hal.h>
//#include "tinyekf.h"

/**
 * @brief Calculates the position of the plane
 *
 */
class Navigation
{
public:
    Navigation(HAL * hal, Plane * plane);
    void update();
    void update_accelerometer();
    void read_imu();
    void read_baro();
    void read_compass();
    void read_gps();
private:
    bool check_new_imu_data();

    HAL * _hal;
    Plane * _plane;
    uint32_t last_imu_timestamp;
    uint32_t last_baro_timestamp;
    uint32_t prev_loop_time;
    uint32_t dt = 10000; // 100Hz
};

#endif
