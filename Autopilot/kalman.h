/*
 * kalman.h
 *
 *  Created on: Jan 1, 2025
 *      Author: jeffr
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "Eigen"

class Kalman
{
public:
    Kalman(int n, int m, Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q);

    void predict(Eigen::MatrixXf u);
    void update(Eigen::MatrixXf R, Eigen::MatrixXf H, Eigen::MatrixXf y);

    Eigen::MatrixXf get_estimate();
    Eigen::MatrixXf get_covariance();

private:
    Eigen::MatrixXf _x;
    Eigen::MatrixXf _P_mat;
    Eigen::MatrixXf _A_mat;
    Eigen::MatrixXf _B_mat;
    Eigen::MatrixXf _Q_mat;
    Eigen::MatrixXf K;

    // State vector: pos_n, pos_e, pos_d, vel_n, vel_e, vel_d
    // Input vector: acc_n, acc_e, acc_d
    int _n = 6; // Length of state vector
    int _m = 3; // Length of input vector

    float predict_dt = 0.01;
    float update_dt = 0.1;
};

#endif /* KALMAN_H_ */
