#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
  void update_Q(float dt);
 public:
  FusionEKF();
  virtual ~FusionEKF();
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  KalmanFilter ekf_;

 private:
  bool is_initialized_;
  long long previous_timestamp_;
  Tools tools;
};

#endif // FusionEKF_H_
