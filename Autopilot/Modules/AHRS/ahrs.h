#ifndef AHRS_H
#define AHRS_H

#include <Lib/MovingAvg/moving_avg.h>
#include "plane.h"
#include "parameters.h"
#include "hal.h"
#include "Lib/Madgwick/madgwick.h"
#include <stdio.h>
#include <math.h>

/**
 * @brief Attitude Heading Reference System
 *
 */
class AHRS
{
public:
    AHRS(HAL* hal, Plane* plane);
    void setup();
    bool set_initial_state();
    void update();
private:
    void update_gyro();
    void update_imu();
    void update_imu_mag();
    void publish_ahrs();
    bool check_new_imu_data();
    bool check_new_compass_data();
    void apply_compass_calibration(float mag_data[3]);
    bool is_accel_reliable();

    Plane* _plane;
    HAL* _hal;

    Madgwick filter;

    static constexpr size_t window_size = 100;
    float window_ax[window_size];
    float window_ay[window_size];
    float window_az[window_size];
    float window_mx[window_size];
    float window_my[window_size];
    float window_mz[window_size];
    MovingAverage avg_ax;
    MovingAverage avg_ay;
    MovingAverage avg_az;
    MovingAverage avg_mx;
	MovingAverage avg_my;
	MovingAverage avg_mz;

	bool initial_state_set = false;

    uint64_t last_imu_timestamp = 0;
    uint64_t last_compass_timestamp = 0;
};

#endif
