#include "tools.h"

using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

  const int n = estimations.size();
  const int m = estimations[0].size();
  VectorXd rmse;
  VectorXd residue;
  rmse.setZero(m);

  for(int i = 0; i < n; i++)
  {
    // Type estimations is std::vector
    residue = estimations[i] - ground_truth[i];
    rmse += residue.array().square().matrix();
  }
  
  return (rmse/n).cwiseSqrt();
}
