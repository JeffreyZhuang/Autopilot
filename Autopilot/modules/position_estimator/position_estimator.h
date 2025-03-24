#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H

#include "lib/kalman/kalman.h"
#include "lib/moving_average/moving_avg.h"
#include "lib/utils/utils.h"
#include "hal.h"
#include "parameters.h"
#include "constants.h"
#include "module.h"
#include <stdio.h>

static constexpr int n = 6;
static constexpr int m = 3;

enum class Pos_estimator_state
{
	INITIALIZATION,
	RUNNING
};

/**
 * @brief Calculates the position and altitude of the plane
 */
class Position_estimator : public Module
{
public:
    Position_estimator(HAL* hal, Plane* plane);

    void update();

private:
    Kalman kalman;

    Pos_estimator_state pos_estimator_state = Pos_estimator_state::INITIALIZATION;

    MovingAverage avg_baro;
    MovingAverage avg_lat;
    MovingAverage avg_lon;
    static constexpr uint8_t window_len = 50;
    float window_baro[window_len];
    float window_lat[window_len];
    float window_lon[window_len];

    Subscription_handle imu_handle;
    Subscription_handle baro_handle;
    Subscription_handle gnss_handle;
    Subscription_handle of_handle;
    Subscription_handle ahrs_handle;

    void update_initialization();
    void update_running();

    void predict_imu();
	void update_gps();
	void update_baro();
	void update_plane();
	void update_of_agl();

    // Kalman
    Eigen::MatrixXf get_a(float dt);
    Eigen::MatrixXf get_b(float dt);
    Eigen::MatrixXf get_q();

	Eigen::Vector3f inertial_to_ned(const Eigen::Vector3f& imu_measurement, float roll, float pitch, float yaw);

	bool is_of_reliable();
};

#endif
