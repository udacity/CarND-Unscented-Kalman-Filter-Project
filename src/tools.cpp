#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}

MatrixXd Tools::NormalizeAngle(MatrixXd &Z_diff, int dim) {
  double_t TWO_PI = 2. * M_PI;
  int width = Z_diff.cols();
  for (int i = 0; i < width; i+=1) {
    while(Z_diff.col(i)(dim) > M_PI) { Z_diff.col(i)(dim) -= TWO_PI; }
    while(Z_diff.col(i)(dim) < -M_PI) { Z_diff.col(i)(dim) += TWO_PI; }
  }
  return Z_diff;
}
