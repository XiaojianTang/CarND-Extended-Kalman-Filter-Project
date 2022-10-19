#include "kalman_filter.h"

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
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
    
  //predict state x_
  x_=F_*x_;

  //predict covariance P_
  P_=F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  MatrixXd y_ = z-H_*x_;
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  MatrixXd I_ = MatrixXd(4,4).Identity(); 

  x_ = x_ + K_*y_;
  P_ = (I_-K_*H_)*P_;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  //calculate h(x) function
  float px=x_[0];
  float py=x_[1];
  float vx=x_[2];
  float vy=x_[3];  
  
  float ro = sqrt(pow(px,2) + pow(py,2));
  float phi = atan2(px,py);
  float ro_dot = (px*vx + py*vy)/ro;

  MatrixXd hx_(3,1);
  hx_ << ro,
         phi,
         ro_dot;   

  MatrixXd y_ = z-hx_;
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  MatrixXd I_ = MatrixXd(4,4).Identity();   

  //normalize angel value in y_
  #include <cmath>
  while(y_[1]<-M_PI)
  {
    y_[1]+=2*M_PI;
  }

  while (y_[1]>M_PI)
  {
    y_[1]-=2*M_PI;
  }

  x_ = x_ + K_*y_;
  P_ = (I_-K_*H_)*P_;
}
