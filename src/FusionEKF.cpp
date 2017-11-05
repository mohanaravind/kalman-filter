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

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;


  std::cout << "constructed";

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  float px, py, vx, vy = 0;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    std::cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {            
      float ro = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      float ro_dot = measurement_pack.raw_measurements_[2];

      // convert from polar to cartesian coordinates
      px = ro * cos(theta);
      py = ro * sin(theta);
      vx = ro_dot * cos(theta);
      vy = ro_dot * sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
      vx = 0;
      vy = 0;
    }

    // set the state with the initial location and zero velocity
    ekf_.x_ << px, py, vx, vy;
    
    // initialize the uncertainity (covariance matrix)
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    // update the timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    std::cout << "Initialized" << endl;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  // update the timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  float noise_ax = 9.0;
  float noise_ay = 9.0;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float dt_4_4 = dt_4 / 4;
  float dt_3_2 = dt_3 / 2;

  // modify the F matrix so that the time is 
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
            0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
            dt_3_2 * noise_ax, 0, dt_2*noise_ax, 0,
            0, dt_3_2 * noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {    
    // init
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;

    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {    
    // init
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
