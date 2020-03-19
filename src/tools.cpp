#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }
  vector<double> residual(4,0);  
  for (size_t i=0; i < estimations[0].size(); i++) {
    for(size_t j = 0; j < estimations.size(); j++){
        residual[i]  +=  pow((estimations[j](i) - ground_truth[j](i)),2);
    }
     residual[i] /= estimations.size();
     residual[i] = sqrt(residual[i]);
  }
  rmse << residual[0], residual[1], residual[2],  residual[3];
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float px2 = pow(px, 2), py2 = pow(py, 2), vx2 = pow(vx, 2), vy2 = pow(vy, 2);
  // check division by zero
  if(abs(py2 + px2) == 0) throw std::runtime_error("Division by zero");
  // TODO: YOUR CODE HERE 
  Hj(0, 0) = px/(pow((px2 + py2),0.5));
  Hj(0, 1) = py/(pow((px2 + py2),0.5));
  Hj(1, 0) = - py/(px2 + py2);
  Hj(1, 1) = px/(px2 + py2);
  Hj(2, 0) = (py * (vx * py - vy * px)) / pow((px2 + py2),1.5);
  Hj(2, 1) = (px * (vy * px - vx * py)) / pow((px2 + py2),1.5);
  Hj(2, 2) = px/(pow((px2 + py2),0.5));
  Hj(2, 3) = py/(pow((px2 + py2),0.5));    
  return Hj;
}
