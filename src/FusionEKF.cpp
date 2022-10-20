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
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.

      // according to main.cpp 
      // ro: measurement_pack.raw_measurements_[0]
      // phi:  measurement_pack.raw_measurements_[1]
      // ro_dot:  measurement_pack.raw_measurements_[2]
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      
      // according to trigonometry:
      // sin(phi) = py/ro -> py = ro*sin(phi) 
      // cos(phi) = px/ro -> px = ro*cos(phi)
      
      //initialize px      
      ekf_.x_(0) = ro*cos(phi);

      //initialize py
      ekf_.x_(1) = ro*sin(phi);    

      //uable to initialize vx,vy from ro_dot            
    }

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.

      //initialize px
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];

      //initialize py
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];

      //unable to initialize vx,vy from lidar measurements
    }

    //initialize timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    
    //initialize covariance matrix P_
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ <<1000,0,0,0,
              0,1000,0,0,
              0,0,1,0,
              0,0,0,1; 

    //define matrix H_laser_    
    H_laser_ << 1,0,0,0,
                0,1,0,0;              
  
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

  //calculate delta_t
  float delta_t = (measurement_pack.timestamp_- previous_timestamp_)/1000000.0;
    
  //update timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  //update matrix F with elapsed time
  ekf_.F_=MatrixXd(4,4);    
  ekf_.F_ << 1,0,delta_t,0,
             0,1,0,delta_t,
             0,0,1,0,
             0,0,0,1;

  //update matrix Q
  float noise_ax = 9;
  float noise_ay = 9;
  ekf_.Q_=MatrixXd(4,4);
  ekf_.Q_ << pow(delta_t,4.0)/4*noise_ax , 0 , pow(delta_t,3.0)/2*noise_ax , 0 ,
               0,  pow(delta_t,4.0)/4*noise_ay , 0 , pow(delta_t,3.0)/2*noise_ay,
               pow(delta_t,3.0)/2*noise_ax , 0 , pow(delta_t,2.0)*noise_ax,0,
               0, pow(delta_t,3.0)/2*noise_ay , 0 ,  pow(delta_t,2.0)*noise_ay;

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
    // TODO: Radar updates  
        
    Hj_= tools.CalculateJacobian(ekf_.x_); 
    ekf_.R_=MatrixXd(3,3);
    ekf_.R_ = R_radar_;

    ekf_.H_=MatrixXd(3,4);
    ekf_.H_ = Hj_;    
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);    


  } else {
    // TODO: Laser updates                    

    ekf_.R_=MatrixXd(2,2);
    ekf_.R_ = R_laser_;

    ekf_.H_=MatrixXd(2,4);
    ekf_.H_ = H_laser_;    
    ekf_.Update(measurement_pack.raw_measurements_); 
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
