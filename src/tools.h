#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "measurement_package.h"
#include "Eigen/Dense"

class Tools {
 public:
  Tools();
  virtual ~Tools();
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  Eigen::VectorXd to_cartesian(const Eigen::VectorXd&);
  Eigen::VectorXd to_polar(const MeasurementPackage&);
};

#endif  // TOOLS_H_
