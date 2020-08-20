#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	/**
	 * TODO: Initialize PID coefficients (and errors, if needed)
	 */
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
	i_error_ = 0.0;
	last_cte_ = 0.0; // remember the last state for difference.

}

void PID::UpdateError(double cte) {
	/**
	 * TODO: Update PID errors based on cte.
	 */
	p_error_ = cte;
	i_error_ += cte;
	d_error_ = cte - last_cte_;
	// Update the last cte:
	last_cte_ = cte;
}

double PID::TotalError() {
	/**
	 * TODO: Calculate and return the total error
	 */
	double total_error = Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
	return total_error;  // TODO: Add your total error calc here!
}

void PID::PrintPIDControl() {
	std::cout << "Kp_ " << Kp_ << std::endl;
	std::cout << "Ki_ " << Ki_ << std::endl;
	std::cout << "Kd_ " << Kd_ << std::endl;

}