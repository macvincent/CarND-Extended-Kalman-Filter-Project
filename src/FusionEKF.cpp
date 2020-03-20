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
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;
    ekf_.R_ << 0.0225, 0,
          0, 0.0225;
    ekf_.H_ << 1, 0, 0, 0,
          0, 1, 0, 0;
    ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
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
    double px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
    double py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
    double vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
    double vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
    VectorXd cordinate_value (4);
    cordinate_value << px, py, vx, vy;
    ekf_.Hj_ = tools.CalculateJacobian(cordinate_value);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
