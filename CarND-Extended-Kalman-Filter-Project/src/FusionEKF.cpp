#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  /**
   * Set the measurement noises and measurement matrix for laser
   */

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
	  0, 1, 0, 0;

  // Hj_ will depend on state, so change value when used. 

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Create the covariance matrix.
     */
    // Initialize its state 
    VectorXd x_in = VectorXd(4);
	x_in << 1, 1, 1, 1;

	// Initialize state cov matrix
	MatrixXd P_in = MatrixXd(4, 4);
	P_in << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

	// Initialize state update matrix
	MatrixXd F_in = MatrixXd(4, 4);

	// Initialize process cov matrix
	MatrixXd Q_in = MatrixXd(4, 4);
	ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);

	// Update timestamp:
	previous_timestamp_ = measurement_pack.timestamp_;

	// Initialize states using the first measurement:
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
	    // Radar measures [rho, phi, rho_dot]=[range, bearing, range rate]
		double rho = measurement_pack.raw_measurements_[0];
		double phi = measurement_pack.raw_measurements_[1];
		double rho_dot = measurement_pack.raw_measurements_[2];
		double px = rho * cos(phi);
		double py = rho * sin(phi);
		double vx = rho_dot * cos(phi);
		double vy = rho_dot * sin(phi);
		ekf_.x_ << px, py, vx, vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
	  // Initialize states using ladar
		// Ladar measurements [px,py]
		ekf_.x_[0] = measurement_pack.raw_measurements_[0];
		ekf_.x_[1] = measurement_pack.raw_measurements_[1];
    }

    // done initializing, no need to predict or update
	// Set flag initizlied 
    is_initialized_ = true;

    return;
  }

  /**
   * If already initialized:
   */

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for the Q matrix.
   */
  // Calculate time difference:
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // [s]
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  previous_timestamp_ = measurement_pack.timestamp_;
	
  /**
  * State prediction eqn: x_{k+1} = F * x_{k} + G * a_k.
  * G * a_{k} has mean 0 and cov Q
  */
  // Update F matrix
  ekf_.F_ << 1, 0, dt, 0,
	  0, 1, 0, dt,
	  0, 0, 1, 0,
	  0, 0, 0, 1;

  // Update process covariance matrix
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  ekf_.Q_ << dt_4 / 4.0*noise_ax, 0, dt_3 / 2.0*noise_ax, 0,
	  0, dt_4 / 4.0*noise_ay, 0, dt_3 / 2.0*noise_ay,
	  dt_3 / 2.0*noise_ax, 0, dt_2*noise_ax, 0,
	  0, dt_3 / 2.0*noise_ay, 0, dt_2*noise_ay;

  // Perform state prediction
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	Hj_ = tools.CalculateJacobian(ekf_.x_); // using predicted x to calculate jacobian
	ekf_.H_ = Hj_;
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else {
    // Laser updates
	// Laser measurement in linear, use standard Kalman Filter
	ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
