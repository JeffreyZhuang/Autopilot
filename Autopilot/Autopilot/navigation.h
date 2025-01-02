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
    HAL * _hal;
    Plane * _plane;
    Kalman kalman;

    uint64_t last_imu_timestamp;
    uint64_t last_gnss_timestamp;
    float predict_dt = 1 / 400;
    float update_dt = 1 / 10;

    float acc_n; // Rotated to the world frame
    float acc_e;
    float acc_d;
    float gnss_n; // meters
    float gnss_e;
    float gnss_d;

    // Kalman
    static constexpr int n = 4; // State vector length
    static constexpr int m = 2; // Input vector length
    Eigen::Matrix<float, n, n> A;
    Eigen::Matrix<float, n, m> B;
    Eigen::DiagonalMatrix<float, n> Q;
    Eigen::DiagonalMatrix<float, m> R;
    Eigen::Matrix <float, m, 1> u;
    Eigen::DiagonalMatrix<float, n> H;
    Eigen::Matrix <float, n, 1> y;

    void read_imu();
	void read_gnss();
	bool check_new_imu_data();
	bool check_new_gnss_data();
};

#endif
