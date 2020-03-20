#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
  P_ = MatrixXd(4, 4);
  R_ = MatrixXd(2, 2);
  R_ << 0.01, 0,
        0, 0.01;
  H_ = MatrixXd(2, 4);
  F_ = MatrixXd(4, 4);
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.01, 0, 0,
              0, 0.0001, 0,
              0, 0, 0.01;
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
}
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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred = tools.to_cartesian(x_);
  VectorXd y = z - z_pred;
  MatrixXd Ht = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
