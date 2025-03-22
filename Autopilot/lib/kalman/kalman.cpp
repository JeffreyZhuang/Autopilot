#include <lib/kalman/kalman.h>

Kalman::Kalman(int n, int m)
{
	_n = n;
	_m = m;
	_P_mat = Eigen::MatrixXf::Zero(n, n);
	_x = Eigen::MatrixXf::Zero(n, 1);
}

void Kalman::predict(Eigen::MatrixXf u,
					 Eigen::MatrixXf A,
					 Eigen::MatrixXf B,
					 Eigen::MatrixXf Q)
{
    _x = A * _x + B * u;
    _P_mat = A * _P_mat * A.transpose() + Q;
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
