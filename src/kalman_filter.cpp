#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  long x_size = x_.rows();
  MatrixXd I_ = MatrixXd::Identity(x_size,x_size);
  VectorXd y = z - H_ * x_;
  VectorXd S_ = H_*P_*H_.transpose() + R_;
  VectorXd K_ = P_*H_.transpose()*S_.inverse();
  x_ = x_ + K_*y;
  P_ = (I_-K_*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Change predicted states to match radar data format
  float rho = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  float phi = atan(x_[1]/x_[0]);
  float rhodot = (x_[0]*x_[2] + x_[1]*x_[3]) / sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  VectorXd h(3);
  h << rho, phi, rhodot;
  VectorXd y = z - h;

  long x_size = x_.rows();
  MatrixXd I_ = MatrixXd::Identity(x_size,x_size);
  VectorXd S_ = H_*P_*H_.transpose() + R_;
  VectorXd K_ = P_*H_.transpose()*S_.inverse();
  x_ = x_ + K_*y;
  P_ = (I_-K_*H_)*P_;
}
