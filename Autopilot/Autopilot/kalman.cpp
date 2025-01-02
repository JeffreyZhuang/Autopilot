#include "kalman.h"

Kalman::Kalman()
{
	_A_mat = Eigen::MatrixXf::Zero(n, n);
	_A_mat << 1, 0, predict_dt, 0,
			  0, 1, 0,	   		predict_dt,
		 	  0, 0, 1, 	   		0,
		 	  0, 0, 0, 	   		1;

	_B_mat = Eigen::MatrixXf::Zero(n, m);
	_B_mat << 0.5*predict_dt*predict_dt, 0,
			  0, 					 	 0.5*predict_dt*predict_dt,
			  predict_dt, 			 	 0,
		 	  0, 					 	 predict_dt;

	_Q_mat = Eigen::MatrixXf::Zero(n, n);
	_Q_mat << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	_R_mat = Eigen::MatrixXf::Zero(m, m);
	_R_mat << 1, 0,
			  0, 1;

	_x = Eigen::MatrixXf::Zero(n, 1);
	_x << 0,
		  0,
		  0,
		  0;

	_P_mat = Eigen::MatrixXf::Zero(n, n);
	_P_mat << 0, 0, 0, 0,
			  0, 0, 0, 0,
			  0, 0, 0, 0,
			  0, 0, 0, 0;
}

void Kalman::predict(Eigen::MatrixXf u)
{
    _x = _A_mat * _x + _B_mat * u;
    _P_mat = _A_mat * _P_mat * _A_mat.transpose() + _Q_mat;
}

void Kalman::update(Eigen::MatrixXf H, Eigen::MatrixXf y)
{
    Eigen::MatrixXf K = _P_mat * H.transpose() * (H * _P_mat * H.transpose() + _R_mat).inverse();
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
