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
  /**
  TODO:
    * predict the state
   */
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd y = z - H_* x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_ ;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;
    
    x_ = x_ + (K * y);
    int n = x_.size();
    MatrixXd I = MatrixXd::Identity(n, n);
    P_ = (I - K * H_) * P_;
    
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
    
    VectorXd h = Eigen::VectorXd(3);
    double r_2 = px * px + py * py;
    h << sqrt(r_2), atan2(py, px), (px * vx + py * vy)/sqrt(r_2);
    
    VectorXd y = z - h;
    // normalizing the angle, phi, y[1]
    while (y[1] > M_PI) {
        y[1] = y[1] - 2 * M_PI;
        std::cout << "normalizing by subtracting 2 pi" << std::endl;
    }
    while (y[1] < -1 * M_PI) {
        y[1] = y[1] + 2 * M_PI;
        std::cout << "normalizing by adding 2 pi" << std::endl;
    }
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_ ;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;
    
    x_ = x_ + (K * y);
    MatrixXd I_ =  MatrixXd::Identity(4, 4);
    P_ = (I_ - K * H_) * P_;
    
    
}
