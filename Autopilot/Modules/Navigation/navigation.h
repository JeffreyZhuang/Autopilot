#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Lib/MovingAvg/moving_avg.h"
#include "Lib/Kalman/kalman.h"
#include "Lib/Utils/utils.h"
#include "hal.h"
#include "parameters.h"
#include "constants.h"
#include <stdio.h>

static constexpr int n = 6;
static constexpr int m = 3;

enum class Nav_state
{
	INITIALIZATION,
	RUNNING
};

/**
 * @brief Calculates the position and altitude of the plane
 */
class Navigation
{
public:
    Navigation(HAL* hal, Plane* plane);
    void update();
private:
    HAL* _hal;
    Plane* _plane;
    Kalman kalman;

    Nav_state nav_state = Nav_state::INITIALIZATION;

    MovingAverage avg_baro;
    MovingAverage avg_lat;
    MovingAverage avg_lon;
    static constexpr uint8_t window_len = 50;
    float window_baro[window_len];
    float window_lat[window_len];
    float window_lon[window_len];

    uint64_t last_imu_timestamp = 0;
    uint64_t last_gnss_timestamp = 0;
    uint64_t last_baro_timestamp = 0;
    uint64_t last_ahrs_timestamp = 0;
    uint64_t last_rangefinder_timestamp = 0;

    void update_initialization();
    void update_running();

    void predict_imu();
	void update_gps();
	void update_baro();
	void update_plane();

    // Kalman
    Eigen::MatrixXf get_a(float dt);
    Eigen::MatrixXf get_b(float dt);
    Eigen::MatrixXf get_q();

	bool check_new_imu_data();
	bool check_new_baro_data();
	bool check_new_gnss_data();
	bool check_new_ahrs_data();
	bool check_new_rangefinder_data();
	Eigen::Vector3f inertial_to_ned(const Eigen::Vector3f& imu_measurement, float roll, float pitch, float yaw);
};

#endif
