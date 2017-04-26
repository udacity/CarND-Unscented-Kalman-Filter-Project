#include <sstream>
#include <iostream>
#include <iomanip>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::string;
using std::stringstream;

Tools::Tools() {}
Tools::~Tools() {}

/*static*/ VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  auto rmse = VectorXd{ 4 };
  rmse << 0.0, 0.0, 0.0, 0.0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cerr << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array(); //coefficient-wise multiplication
    rmse += residual;
  }

  rmse = rmse / (double)estimations.size();       //calculate the mean
  rmse = rmse.array().sqrt();                     //calculate the squared root

  return rmse;
}

/*static*/ string Tools::PrintMatrix(const MatrixXd& m, const char FS) {
  stringstream sstr;
  sstr << std::setprecision(3);
  for (auto row = 0; row < m.rows(); ++row)
  {
    sstr << m(row, 0);
    for (auto col = 1; col < m.cols(); ++col)
      sstr << FS << m(row, col);
    sstr << std::endl;
  }
  return sstr.str();
}
