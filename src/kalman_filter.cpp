#include "kalman_filter.h"
#include "tools.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_*x_ ;
  P_ = F_*P_*(F_.transpose()) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  
  VectorXd y(2);

  MatrixXd S(2,2);
  MatrixXd K(4,2);
  MatrixXd I(4,4);

  // KF Measurement update step
  y = z- (H_*x_); 
  S = H_ * P_ * H_.transpose() +R_; //what happens when S is not invertible?
  if(S.determinant()==0)
  {
    S << 1,0,0,1;
    cout << "S not invertible" << endl;
  }
     
  K = P_ * H_.transpose() * S.inverse();
  I = MatrixXd::Identity(4,4);//idetntity of the appropriate size

  // new state
 
  x_ = x_ + K*y;
  P_ = (I - K*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd hx(3);
  VectorXd y(3);
  MatrixXd S(3,3);
  MatrixXd K(4,3);
  MatrixXd I(4,4);
  float rho,phi,rho_dot;
  rho = sqrt( x_(0)*x_(0) + x_(1)*x_(1) );
  float a = x_(1);
  float b = x_(0);
  if(0<=b && b< 0.0001 )
  {
    b=0.0001;
    cout << "b < 0.0001";
  }
  else if(-0.0001<b && b<0)
  {
    b=-0.0001;
  }
  phi = atan2(a,b);// 
  if(rho<0.0001)
  {
    rho = 0.001;
    cout << "rho< 0.0001";
  }
  rho_dot = (x_(0)*x_(2) + x_(1)*x_(3) )/rho;
  hx << rho, phi, rho_dot;

  y = z-hx;

  //make sure that the phi component of y is in the range of -pi and pi (by adding/subtracting  2pi)
  while(y(1)>M_PI)
  {
    y(1)-=2*M_PI;
    cout << "y(1) > pi"<< endl;
  }
  while(y(1)< - M_PI)
  {
    y(1) += 2*M_PI;
    cout << "y(1) < pi " << endl;
  }
 
  
  
  //Tools tool1;
  S = H_ * P_ * H_.transpose() +R_;
  if(S.determinant()==0)
  {
    S << 1,0,0,0,1,0,0,0,1;
    cout << "EKF: S not invertible" << endl;
  }
  K = P_ * H_.transpose() * S.inverse();
  I = MatrixXd::Identity(4,4);//idetntity of the appropriate size
  // new state
  x_ = x_ + K*y;
  P_ = (I - K*H_) * P_;

}
