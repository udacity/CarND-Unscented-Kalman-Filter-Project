#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
    /**
     * A helper method to calculate RMSE
     */
    static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
    
    /**
     * Ensure that input angle is between -PI and PI.
     * @param angle input angle
     * @return normalized angle
     */
    static double NormalizeAngle(const double angle);
    
    /**
     * Calculate weights for sigma point calculation in Unscented Kalman Filter
     * @param n_aug Number of augmented states
     * @param lambda Tuning parameter (Default: 3.0 - n_aug)
     * @return vector with weights
     */
    static Eigen::VectorXd calculateWeights(const int n_aug, const double lambda);
    
    /**
     * Calculate Normal Innovation Squared measurement (NIS)
     * @param z_diff Difference in measurement space
     * @param S Predicted measurement covariance
     * @return NIS value
     */
    static double calculateNIS(const Eigen::VectorXd& z_diff, const Eigen::MatrixXd &S);
};

#endif /* TOOLS_H_ */
