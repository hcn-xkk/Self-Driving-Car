#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  std::cout << "Debug: Init function" << std::endl;
  std::cout << "H_in: " << H_in << std::endl;
  std::cout << "x_in: " << x_in << std::endl;

}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state mean and cov
   */
	// x_{k+1} = F * x_{k} + G * a_k.
	// G * a_{k} has mean 0 and cov Q
	// the initial transition matrix F_
	x_ = F_ * x_;
	P_ = F_ * P_ * (F_.transpose()) + Q_;

}

void KalmanFilter::PrepareErrorForKFUpdate(const VectorXd &z) {
  /**
   * Generate error vector y using ladar measurement
   * for updating step in Kalman Filter equations
   */
   // Calculate error y
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	
	// Measurement update step in Kalman Filter
	UpdateWithError(y);
}

void KalmanFilter::PrepareErrorForEKFUpdate(const VectorXd &z) {
  /**
   * Generate error vector y using radar measurement
   * for updating step in Extended Kalman Filter equations
   * This function requires the radar measurement function z = h(x)
   */
	// z_pred using h(x)
	VectorXd z_pred(z.size()); // Radar measurement [rho, phi, rho_dot]
	double px = x_[0];
	double py = x_[1];
	double vx = x_[2];
	double vy = x_[3];
	double rho = sqrt(px * px + py * py);
	z_pred[0] = rho;
	z_pred[1] = atan2(py, px); // atan2 returns value in -pi, pi
	z_pred[2] = (px * vx + py * vy) / rho;
	
	// Calculate error y
	VectorXd y = z - z_pred;
	// y[1] is angle, make sure angle is within [-pi,pi]
	while (y(1) > M_PI || y(1) < -M_PI) {
		if (y(1) > M_PI) {
			y(1) -= M_PI;
		}
		else {
			y(1) += M_PI;
		}
	}
	
	// Measurement update step in Kalman Filter
	UpdateWithError(y);
}

void KalmanFilter::UpdateKFWithError(const VectorXd &y) {

	// Measurement update step in Kalman Filter
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