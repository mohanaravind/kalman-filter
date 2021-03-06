#include "kalman_filter.h"
#include <iostream>

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
  // predict the state 
  x_ = F_ * x_;

  // compute the uncertainity
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {    
  // compute the error
  VectorXd y = z - H_ * x_;
  
  ComputeKF(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // convert the previous state to polar co-ordinate
  float ro = sqrt(px * px + py * py);
  float theta = atan2(py, px);
  float ro_dot = (px * vx + py * vy) / ro;

  // if px and py are really low then set theta and ro_dot to 0
  if (fabs(px) < 0.0001 && fabs(py) < 0.0001) {
    theta, ro_dot = 0;
  }

  VectorXd hx = VectorXd(3);
  hx << ro, theta, ro_dot;

  // compute the error
  VectorXd y = z - hx;

  // normalize the angle
  // Reference: https://discussions.udacity.com/t/ekf-gets-off-track/276122/26?u=aravinde4e025b6ca72e
  y[1] = atan2(sin(y[1]), cos(y[1]));
  
  ComputeKF(y);
}

void KalmanFilter::ComputeKF(const Eigen::VectorXd & error)
{
  // compute the kalman weight
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // update the state
  x_ = x_ + (K * error);

  // compute the uncertainity for this state
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
