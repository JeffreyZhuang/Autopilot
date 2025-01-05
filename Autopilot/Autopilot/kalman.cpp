#include "kalman.h"

Kalman::Kalman(int n, int m, Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q, Eigen::MatrixXf R)
{
	_n = n;
	_m = m;
	_A_mat = A;
	_B_mat = B;
	_Q_mat = Q;
	_R_mat = R;

	_P_mat = Eigen::MatrixXf::Zero(n, n);
	_x = Eigen::MatrixXf::Zero(n, 1);
}

void Kalman::reset()
{
	_x = Eigen::MatrixXf::Zero(_n, 1);
}

void Kalman::predict(Eigen::MatrixXf u)
{
    _x = _A_mat * _x + _B_mat * u;
    _P_mat = _A_mat * _P_mat * _A_mat.transpose() + _Q_mat;
}

void Kalman::update(Eigen::MatrixXf H, Eigen::MatrixXf y)
{
    K = _P_mat * H.transpose() * (H * _P_mat * H.transpose() + _R_mat).inverse();
    _x = _x + K * (y - H * _x);
    _P_mat = _P_mat - K * H * _P_mat;
}

Eigen::MatrixXf Kalman::get_estimate()
{
    return _x;
}

Eigen::MatrixXf Kalman::get_covariance()
{
    return _P_mat;
}
