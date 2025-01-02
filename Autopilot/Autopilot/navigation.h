#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "hal.h"
#include "kalman.h"

/**
 * @brief Calculates the position of the plane
 *
 */
class Navigation
{
public:
    Navigation(HAL * hal, Plane * plane);
    void execute();
    void prediction_step();
    void update_step();
private:
    void read_imu();
    void read_gnss();
    bool check_new_imu_data();
    bool check_new_gnss_data();

    HAL * _hal;
    Plane * _plane;
    uint64_t last_imu_timestamp;
    uint64_t last_gnss_timestamp;
    uint64_t time;

    // Earth centered frame
    float acc_n;
    float acc_e;
    float acc_d;

    // Meters
    float gnss_n;
    float gnss_e;
    float gnss_d;
};

#endif
