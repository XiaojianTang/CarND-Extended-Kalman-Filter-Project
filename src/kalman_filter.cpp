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

  return;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  VectorXd y_ = z-H_*x_;
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  MatrixXd I_(4,4);
  I_ << 1,0,0,0,
      0,1,0,0,
      0,0,1,0,
      0,0,0,1;

  x_ = x_ + K_*y_;
  P_ = (I_-K_*H_)*P_;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  //calculate h(x) function
  float px=x_(0);
  float py=x_(1);
  float vx=x_(2);
  float vy=x_(3); 
  
  VectorXd hx_(3);
  
  if(fabs(pow(px,2.0) + pow(py,2.0))>0.0001 and fabs(py)>0.0001){  
    float ro = sqrt(pow(px,2.0) + pow(py,2.0));
    float phi = atan2(py,px);
    float ro_dot = (px*vx + py*vy)/ro;
  
    hx_ << ro, phi, ro_dot;}   

  VectorXd y_ = z-hx_;
  MatrixXd S_ = H_*P_*H_.transpose() + R_;
  MatrixXd K_ = P_*H_.transpose()*S_.inverse();
  MatrixXd I_(4,4);
  I_ << 1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;

  //normalize angel value in y_  
  if(y_(1)<-M_PI)
  {
    y_(1)=y_(1)+2*M_PI;
  }

  else if (y_(1)>M_PI)
  {
    y_(1)=y_(1)-2*M_PI;
  }

  x_ = x_ + K_*y_;
  P_ = (I_-K_*H_)*P_;
}
