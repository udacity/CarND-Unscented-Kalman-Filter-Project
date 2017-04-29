#include <sstream>
#include <iostream>
#include <iomanip>
#include <numeric>
#include "tools.h"

using Eigen::Vector2d;
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

/* static */ VectorXd Tools::CalculateStdDev(const std::vector<VectorXd>& process_state) {
  // standard deviation of population. ref https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  VectorXd retVal;
  auto N = process_state.size();
  if (N < 1) return retVal;
  auto K = process_state[0];
  retVal = VectorXd(process_state[0].rows()); retVal.fill(0.0);
  auto sum = retVal; auto sum2 = retVal;
  for (auto i = 0u; i < N; ++i) {
    auto d = process_state[i] - K;
    sum += d; 
    sum2 += d.cwiseProduct(d);
  }
  retVal = (sum2 - (sum.cwiseProduct(sum) / N)) / N; // (n-1) for sample
  retVal = retVal.array().sqrt();
  return retVal;
}
