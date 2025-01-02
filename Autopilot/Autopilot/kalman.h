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
    Kalman(int n, int m);
    void set_matrices(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q, Eigen::MatrixXf R);
    void predict(Eigen::MatrixXf u);
    void update(Eigen::MatrixXf H, Eigen::MatrixXf y);
    Eigen::MatrixXf get_estimate();
    Eigen::MatrixXf get_covariance();

private:
    Eigen::MatrixXf _x;
    Eigen::MatrixXf _P_mat;
    Eigen::MatrixXf _A;
    Eigen::MatrixXf _B_mat;
    Eigen::MatrixXf _Q;
    Eigen::MatrixXf _R;
    int _n; // State vector length
    int _m; // Input vector length
};

#endif /* KALMAN_H_ */
