#ifndef KALMAN_H_
#define KALMAN_H_

#include "lib/eigen/Eigen/Eigen"

class Kalman
{
public:
    Kalman(int n, int m);

    void predict(Eigen::MatrixXf u, Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q);
    void update(Eigen::MatrixXf R, Eigen::MatrixXf H, Eigen::MatrixXf y);
    Eigen::MatrixXf get_estimate();
    Eigen::MatrixXf get_covariance();
    void set_x(Eigen::MatrixXf x);

private:
    Eigen::MatrixXf _x;
    Eigen::MatrixXf _P_mat;
    Eigen::MatrixXf K;
    int _n; // Length of state vector
    int _m; // Length of input vector
};

#endif /* KALMAN_H_ */
