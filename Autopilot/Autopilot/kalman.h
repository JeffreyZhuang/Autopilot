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
    Kalman(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R);

    void predict(Eigen::VectorXd u);
    void update(Eigen::MatrixXd H, Eigen::VectorXd y);
    Eigen::VectorXd getEstimate();
    Eigen::VectorXd getCovariance();

private:
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
};

#endif /* KALMAN_H_ */
