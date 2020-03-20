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
  R_laser_ << 0.01, 0,
              0, 0.01;

  //measurement covariance matrix - radar
  R_radar_ << 0.01, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.01;
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    cout << "EKF: " <<  endl;
    ekf_.x_ = VectorXd(4);
    if(measurement_pack.sensor_type_ == MeasurementPackage::LASER) ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 4, 1;
    else ekf_.x_ = tools.to_polar(measurement_pack);
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }
  double noise_ax = 9;
  double noise_ay = 9;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << (pow(dt,4)/4)*noise_ax, 0, (pow(dt,3)/2)*noise_ax, 0,
              0, (pow(dt,4)/4)*noise_ay, 0, (pow(dt,3)/2)*noise_ay,
              (pow(dt,3)/2)*noise_ax, 0, pow(dt,2)*noise_ax, 0,
              0, (pow(dt,3)/2)*noise_ay, 0, pow(dt,2)*noise_ay;
  ekf_.Predict();
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
}
