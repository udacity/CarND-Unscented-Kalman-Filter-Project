#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
    /**
     * A helper method to calculate RMSE.
     */
    static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
    
    /**
     * Ensure that input angle is between -PI and PI.
     */
    static double NormalizeAngle(const double angle);
    
    /**
     *
     */
    static Eigen::VectorXd calculateWeights(const int n_aug, const double lambda);
    
    /**
     *
     */
    static double calculateNIS(const Eigen::VectorXd& z_diff, const Eigen::MatrixXd &S);
    
};

#endif /* TOOLS_H_ */
