#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
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
}

void KalmanFilter::Predict() {
	std::cout << "pred" << std::endl;
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	 * TODO: update the state by using Kalman Filter equations
	 */
	std::cout << "l" << std::endl;
	VectorXd z_pred = H_ * x_;
	std::cout << "l2" << std::endl;
	VectorXd y = z - z_pred;
	std::cout << "l3" << std::endl;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	std::cout << "l4" << std::endl;
	x_ = x_ + (K * y);
	std::cout << "l5" << std::endl;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

VectorXd KalmanFilter::radar_H(const VectorXd &x_state) {
	VectorXd z_pred = VectorXd(3);
	
	z_pred(0) = sqrt(x_state(0) * x_state(0) + x_state(1) * x_state(1)); // Rho

	if (x_state(1) != 0) {
		z_pred(1) = tanh2(x_state(0) / x_state(1)); // Phi
	} else {
		//poop
	}

	z_pred(2) = sqrt(x_state(2) * x_state(2) + x_state(3) * x_state(3)); // Rhodot

	return z_pred;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	 * TODO: update the state by using Extended Kalman Filter equations
	 */
	std::cout << "r1" << std::endl;
	VectorXd z_pred = radar_H(x_);
	std::cout << "r2" << std::endl;
	VectorXd y = z - z_pred;
	std::cout << "r3" << std::endl;
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
