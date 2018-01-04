#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::BasicUpdate(const VectorXd &y)
{
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	BasicUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	double px = x_(0);
	double py = x_(1);
	double vx = x_(2);
	double vy = x_(3);
	double px2 = px * px;
	double py2 = py * py;
	double rho = sqrt(px2 + py2);
	const double TWO_PI = 2.0 * M_PI;
	double phi = atan2(py, px);

	double drho = (px * vx + py * vy) / rho;
	VectorXd z_pred = VectorXd(3);
	z_pred << rho, phi, drho;
	VectorXd y = z - z_pred;
	phi = y(1);
	if (phi < -M_PI)
	{
		int add_2pi = (int)((-phi + M_PI) / TWO_PI);
		phi += add_2pi * TWO_PI;
	}
	else if (phi > M_PI)
	{
		int sub_2pi = (int)((phi + M_PI) / TWO_PI);
		phi -= sub_2pi * TWO_PI;
	}
	y(1) = phi;

	BasicUpdate(y);
}
