#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0)
    {
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }
    
    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i)
    {
        
        VectorXd residual = estimations[i] - ground_truth[i];
        
        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    
    //calculate the mean
    rmse = rmse/estimations.size();
    
    //calculate the squared root
    rmse = rmse.array().sqrt();
    
    //return the result
    return rmse;
}

double Tools::NormalizeAngle(const double angle)
{
    double result = angle;
    
    while (result > M_PI)
    {
        result -= 2. * M_PI;
    }
    
    while (result < -M_PI)
    {
        result += 2.*M_PI;
    }
    
    return result;
}

VectorXd Tools::calculateWeights(const int n_aug, const double lambda)
{
    VectorXd weights = VectorXd(2*n_aug+1);
    
    weights(0) = lambda/(lambda+n_aug);
    for (int i=1; i<2*n_aug+1; i++)
    {
        weights(i) = 0.5/(lambda+n_aug);
    }
    
    return weights;
}

double Tools::calculateNIS(const VectorXd& z_diff, const MatrixXd &S)
{
    return (z_diff.transpose()) * (S.inverse()) * z_diff;
}
