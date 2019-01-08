#include "tools.h"

using Eigen::VectorXd;
using std::vector;

using namespace std;
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse;
  VectorXd residue;
  int n = estimations.size();
  int m = estimations[0].size();
  rmse.setZero(m);
  for(int i; i < n; i++){
    residue = estimations[i] - ground_truth[i];
    //std::cout<<"size of residue:\n"<<residue.size();
    rmse += residue.array().square().matrix();
  }
  return (rmse/n).cwiseSqrt();
}
