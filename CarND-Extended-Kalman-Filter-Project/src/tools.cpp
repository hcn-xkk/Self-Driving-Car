#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	int n_estimation = estimations.size();
	int n_truth = ground_truth.size();
	if (n_estimation == 0) {
		cout << "Length of estimation is 0." << endl;
		return rmse;
	}
	if (n_truth != n_estimation) {
		cout << "Length of estimation is not equal to ground truth." << endl;
		return rmse;
	}

	// accumulate squared residuals
	ArrayXd squared_error = rmse.array()*0.0;  // initialize squared error as a sum array.
	for (int i = 0; i < estimations.size(); ++i) {
		// ... your code here
		VectorXd error_i = estimations[i] - ground_truth[i];
		squared_error = squared_error + error_i.array().square();
	}
	cout << "Squared error is : " << squared_error << endl;

	// calculate the mean and squared root
	ArrayXd mean_squared_error = squared_error / estimations.size();
	rmse = mean_squared_error.sqrt().matrix();
	
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
	MatrixXd Hj(3, 4);
	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// TODO: YOUR CODE HERE 
	// check division by zero
	if ((px < 1e-4) && (py < 1e-4)) {
		cout << "Error - Division by zero" << endl;
		return Hj;
	}
	// compute the Jacobian matrix
	float rou_2 = pow(px, 2) + pow(py, 2);
	float rou = sqrt(rou_2);
	float rou_3 = rou * rou_2;
	Hj(0, 0) = px / rou;
	Hj(0, 1) = py / rou;
	Hj(1, 0) = -py / rou_2;
	Hj(1, 1) = px / rou_2;
	Hj(2, 0) = py * (vx*py - vy * px) / rou_3;
	Hj(2, 1) = px * (vy*px - vx * py) / rou_3;
	Hj(2, 2) = px / rou;
	Hj(2, 3) = py / rou;

	return Hj;
}
