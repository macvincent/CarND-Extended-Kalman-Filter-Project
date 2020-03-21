#include <cmath>
#include <iostream>
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;
using std::ifstream;
using std::ofstream;
using std::istringstream;
using std::cout;
using std::endl;

int main() {
  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(), ifstream::in);
  ofstream simulation_data("../data/filter_output.txt");
  ofstream rmse_data("../data/rmse_data.txt");

  if (!in_file.is_open()) cout << "Cannot open input file: " << in_file_name_ << endl;
  if (!rmse_data.is_open()) cout << "Cannot open input file: rsme.txt" << endl;
  if (!simulation_data.is_open()) cout << "Cannot open input file: simulation_data.txt" << endl;

  string line;
  while (getline(in_file, line)) {
    MeasurementPackage meas_package;
    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type; // reads first element from the current line
    int64_t timestamp;

    if (sensor_type.compare("L") == 0) {  // laser measurement
      // read measurements
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x,y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;

    } else if (sensor_type.compare("R") == 0) {
        // read measurements
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        double rho_measured;
        double phi_measured;
        double rhodot_measured;
        iss >> rho_measured;
        iss >> phi_measured;
        iss >> rhodot_measured;
        meas_package.raw_measurements_ << rho_measured,phi_measured,rhodot_measured;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    }
    fusionEKF.ProcessMeasurement(meas_package);
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt; 
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    ground_truth.push_back(gt_values);
    estimations.push_back(fusionEKF.ekf_.x_);
    simulation_data << fusionEKF.ekf_.x_[0] << " " << fusionEKF.ekf_.x_[1] << endl;
  }
  VectorXd rmse = tools.CalculateRMSE(estimations,ground_truth);
  cout << "RMSE: " << rmse << endl;
  rmse_data << rmse;
  
  if (in_file.is_open()) in_file.close();
  if(simulation_data.is_open())simulation_data.close();
  if(rmse_data.is_open())rmse_data.close();

  return 0;
}