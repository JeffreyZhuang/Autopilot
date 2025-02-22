#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "hal.h"
#include "Lib/Kalman/kalman.h"
#include "Lib/Utils/utils.h"
#include "Lib/MovingAverage/moving_avg.h"
#include "parameters.h"

static constexpr int n = 6;
static constexpr int m = 3;

/**
 * @brief Calculates the position of the plane
 *
 */
class Navigation
{
public:
    Navigation(HAL* hal, Plane* plane, float predict_dt);

    void execute();
    bool set_home();

private:
    HAL* _hal;
    Plane* _plane;
    Kalman kalman;

    MovingAverage avg_baro;
    MovingAverage avg_lat;
    MovingAverage avg_lon;
    static constexpr uint8_t window_len = 50;
    float window_baro[window_len];
    float window_lat[window_len];
    float window_lon[window_len];

    bool home_set = false;

    uint64_t last_imu_timestamp = 0;
    uint64_t last_gnss_timestamp = 0;
    uint64_t last_baro_timestamp = 0;

    const float g = 9.80665;

    // Kalman
    Eigen::MatrixXf get_a(float dt);
    Eigen::MatrixXf get_b(float dt);
    Eigen::MatrixXf get_q();

	bool check_new_imu_data();
	bool check_new_baro_data();
	bool check_new_gnss_data();

	void predict_imu();

	void update_gps();
	void update_baro();

	void update_plane();

	Eigen::Vector3f rotateToNED(const Eigen::Vector3f& imu_measurement, float roll, float pitch, float yaw);
};

#endif
