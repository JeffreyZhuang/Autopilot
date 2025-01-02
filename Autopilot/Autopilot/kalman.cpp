#include "kalman.h"

Kalman::Kalman(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R)
{
    this->A = A;
    this->B = B;
    this->Q = Q;
    this->R = R;
    this->x = Eigen::VectorXd::Zero(A.rows());
    P = Eigen::MatrixXd::Zero(A.rows(), A.rows());
}

void Kalman::predict(Eigen::VectorXd u)
{
    x = A * x + B * u;
    P = A * P * A.transpose() + Q;
}

void Kalman::update(Eigen::MatrixXd H, Eigen::VectorXd y)
{
    Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x = x + K * (y - H * x);
    P = P - K * H * P;
}

Eigen::VectorXd Kalman::getEstimate()
{
    return x;
}

Eigen::VectorXd Kalman::getCovariance()
{
    return P;
}
