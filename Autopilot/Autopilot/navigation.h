#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "hal.h"
#include "kalman.h"

enum class NavigationState {
    INITIALIZATION = 1,
    LIVE = 2
};

static constexpr int n = 6;
static constexpr int m = 3;
static constexpr float predict_dt = 0.01;

/**
 * @brief Calculates the position of the plane
 *
 */
class Navigation
{
public:
    Navigation(HAL* hal, Plane* plane);

    void execute();

private:
    HAL* _hal;
    Plane* _plane;
    Kalman kalman;

    NavigationState navigationState = NavigationState::INITIALIZATION;

    void execute_initialization();
    void execute_live();

    uint64_t last_imu_timestamp;
    uint64_t last_gnss_timestamp;
    uint64_t last_baro_timestamp;

    float gnss_n; // meters
    float gnss_e;
    float gnss_d;

    float g = 9.80665;

    // Kalman
    Eigen::MatrixXf get_a(float dt);
    Eigen::MatrixXf get_b(float dt);
    Eigen::MatrixXf get_q();

    void read_imu();
	void read_gnss();

	bool check_new_imu_data();
	bool check_new_baro_data();
	bool check_new_gnss_data();

	void predict_imu();

	void update_gps();
	void update_baro();

	void update_plane();
};

#endif
