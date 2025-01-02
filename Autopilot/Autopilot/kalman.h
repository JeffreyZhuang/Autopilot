/*
 * kalman.h
 *
 *  Created on: Jan 1, 2025
 *      Author: jeffr
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "Eigen"

// State vector: pos_n, pos_e, vel_n, vel_e
// Input vector: acc_n, acc_e
static constexpr int n = 4; // Length of state vector
static constexpr int m = 2; // Length of input vector

class Kalman
{
public:
    Kalman();
    void predict(Eigen::MatrixXf u);
    void update(Eigen::MatrixXf H, Eigen::MatrixXf y);
    Eigen::MatrixXf get_estimate();
    Eigen::MatrixXf get_covariance();

private:
    Eigen::MatrixXf _x;
    Eigen::MatrixXf _P_mat;
    Eigen::MatrixXf _A_mat;
    Eigen::MatrixXf _B_mat;
    Eigen::MatrixXf _Q_mat;
    Eigen::MatrixXf _R_mat;

    float predict_dt = 0.0025;
    float update_dt = 0.1;
};

#endif /* KALMAN_H_ */
