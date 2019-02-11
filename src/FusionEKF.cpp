#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises

  */
  H_laser_ << 1, 0, 0 ,0,
  			0, 1, 0, 0;
  // Hj not explicitly initialized

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf_.x_ <<measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]),
        measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]), 
      measurement_pack.raw_measurements_[2]*cos(measurement_pack.raw_measurements_[1]),
        measurement_pack.raw_measurements_[2]*sin(measurement_pack.raw_measurements_[1]);
      
	  ekf_.R_ = R_radar_;
      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;
      
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      
    }
    ekf_.P_= MatrixXd::Identity(4,4);
    ekf_.P_(3,3)=1000;
    ekf_.P_(2,2)=1000; // high values for velocity, because these values were not initialized with the measurements
    // done initializing, no need to predict or update
	previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    cout << "initialized";
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //long long prev=previous_timestamp_;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  float noise_ax = 9;
  float noise_ay = 9;
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, dt, 0,
  			0, 1, 0, dt,
  			0, 0, 1, 0,
  			0, 0, 0, 1;
  float dt2 = pow(dt,2);
  float dt3 = pow(dt,3);
  float dt4 = pow(dt,4);
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt4*noise_ax/4, 0, dt3*noise_ax/2, 0,
			0, dt4*noise_ay/4, 0, dt3*noise_ay/2,
			dt3*noise_ax/2, 0, dt2*noise_ax, 0,
			0, dt3*noise_ay/2, 0, dt2*noise_ay;				
  
  
  

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
