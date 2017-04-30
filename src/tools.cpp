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
	VectorXd rmse(4);
	rmse << 0,0,0,0;
	if(estimations.size() == 0)
	{
	    std::cout<<"Error"<<std::endl;
	    return rmse;
	}
	else if(estimations.size() != ground_truth.size())
	{
	    std::cout<<"Error"<<std::endl;
	    return rmse;
	}


	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i)
  {
    // ... your code here
    VectorXd diff = (estimations[i] - ground_truth[i]);
		diff = diff.array() * diff.array();
		rmse += diff;
    std::cout<<"Estimations : "<<estimations[i]<<std::endl;
    std::cout<<"Ground Truth : "<<ground_truth[i]<<std::endl;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}
