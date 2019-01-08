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

  /**
    Because of the old version of eigen, we can't use vector operations.
  */
  const int n = estimations.size();
  VectorXd rmse;
  VectorXd residue;
  for(int i = 0; i < n; i++)
  {
    // Type estimations is std::vector
    residue = estimations[i] - ground_truth[i]; 
    
  }
  
}
