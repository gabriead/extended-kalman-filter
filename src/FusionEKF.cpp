#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>

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
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  Hj_ = MatrixXd(3, 4);
  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1; 

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  noise_ax = 9;
  noise_ay = 9;
  
    MatrixXd P_ = MatrixXd(4, 4);
  	P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;
    
  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  
  ekf_.F_ = F_;
  ekf_.P_= P_;

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
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;

    ekf_.x_ << 1,1,1,1;
    
 
  // set the acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;
  
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
	  // transform z coordinates to cartesian coordinates 
      
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float ro_dot = measurement_pack.raw_measurements_[2];
     
      float px = cos(phi)*ro;
      float py = sin(phi)*ro;
      float vx = cos(phi)*ro_dot;
      float vy = sin(phi)*ro_dot;
       
      ekf_.x_ << px,py,vx,vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  	previous_timestamp_ = measurement_pack.timestamp_;
  
   int noise_ax = 9;
   int noise_ay = 9;
  
   MatrixXd F_ = MatrixXd(4, 4);
   F_ << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
  
   ekf_.F_ = F_;
  
    // 2. Set the process covariance matrix Q
  double delta_t_4th_power_sigma_ax = pow(dt,4)/4*noise_ax;
  double delta_t_3rd_power_sigma_ax = pow(dt,3)/2*noise_ax;
  double delta_t_2nd_power_sigma_ax = pow(dt,2)  *noise_ax;
  
  double delta_t_4th_power_sigma_ay = pow(dt,4)/4*noise_ay;
  double delta_t_3rd_power_sigma_ay = pow(dt,3)/2*noise_ay;
  double delta_t_2nd_power_sigma_ay = pow(dt,2)  *noise_ay;
  
  MatrixXd Q_ = MatrixXd(4, 4);
  Q_ << delta_t_4th_power_sigma_ax, 0, delta_t_3rd_power_sigma_ax, 0,
            0, delta_t_4th_power_sigma_ay, 0, delta_t_3rd_power_sigma_ay,
            delta_t_3rd_power_sigma_ax, 0, delta_t_2nd_power_sigma_ax,0,
            0,delta_t_3rd_power_sigma_ay,0, delta_t_2nd_power_sigma_ay;
  
  ekf_.Q_ = Q_;
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates => extended Kalman Filter
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates => regular Kalman Filter
     ekf_.H_ = H_laser_;
     ekf_.R_ = R_laser_;
     ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
