#include "kalman.h"

Kalman::Kalman(int n, int m, Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf Q)
{
	_n = n;
	_m = m;
	_A_mat = A;
	_B_mat = B;
	_Q_mat = Q;

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

void Kalman::update(Eigen::MatrixXf R, Eigen::MatrixXf H, Eigen::MatrixXf y)
{
	Eigen::MatrixXf S = H * _P_mat * H.transpose() + R;
    K = _P_mat * H.transpose() * S.inverse();
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
