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
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    cout << "EKF: Position states stored in file '../data/filter_output.txt' in format (px, py)."<< endl;;
    if(measurement_pack.sensor_type_ == MeasurementPackage::LASER) ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 4, 1;
    else ekf_.x_ = tools.to_polar(measurement_pack);
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  update_Q(dt);

  // Predict
  ekf_.Predict();

  // Update Measurement
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
}

void FusionEKF::update_Q(float dt){
  double noise_ax = 9;
  double noise_ay = 9;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  ekf_.Q_ << (pow(dt,4)/4)*noise_ax, 0, (pow(dt,3)/2)*noise_ax, 0,
              0, (pow(dt,4)/4)*noise_ay, 0, (pow(dt,3)/2)*noise_ay,
              (pow(dt,3)/2)*noise_ax, 0, pow(dt,2)*noise_ax, 0,
              0, (pow(dt,3)/2)*noise_ay, 0, pow(dt,2)*noise_ay;
}
