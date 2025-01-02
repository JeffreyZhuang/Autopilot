#include "kalman.h"

Kalman::Kalman(int n, int m)
{
	_n = n;
	_m = m;
}

void Kalman::set_matrices(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q, Eigen::MatrixXf R)
{
	_A = A;
	_B_mat = B;
	_Q = Q;
	_R = R;
	_x = Eigen::MatrixXf::Zero(_n, 1);
	_P_mat = Eigen::MatrixXf::Zero(_n, _n);
}

void Kalman::predict(Eigen::MatrixXf u)
{
    _x = _A * _x + _B_mat * u;
    _P_mat = _A * _P_mat * _A.transpose() + _Q;
}

void Kalman::update(Eigen::MatrixXf H, Eigen::MatrixXf y)
{
	printf("4\n");
    Eigen::MatrixXf K = _P_mat * H.transpose() * (H * _P_mat * H.transpose() + _R).inverse();
    printf("5\n");
    _x = _x + K * (y - H * _x);
    printf("6\n");
    _P_mat = _P_mat - K * H * _P_mat;
    printf("7\n");
}

Eigen::MatrixXf Kalman::get_estimate()
{
    return _x;
}

Eigen::MatrixXf Kalman::get_covariance()
{
    return _P_mat;
}
