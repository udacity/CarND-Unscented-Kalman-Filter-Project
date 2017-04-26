#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  Tools();
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
  static void NormalizeAngle(double& a) { while (a > M_PI) a -= 2.*M_PI; while (a <-M_PI) a += 2.*M_PI; }
  static std::string PrintMatrix(const Eigen::MatrixXd& m, const char FS='\t');
};

#endif /* TOOLS_H_ */
