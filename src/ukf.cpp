#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;
    
    // initial state vector
    x_ = VectorXd(5);
    
    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;
    
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
    
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;
    
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    
    /**
     TODO:
     
     Complete the initialization. See ukf.h for other member properties.
     
     Hint: one or more values initialized above might be wildly off...
     */
    
    ///* State dimension
    n_x_ = 5;
    
    ///* Augmented state dimension
    n_aug_ = 7;
    
    ///* Sigma point spreading parameter
    lambda_ = 3 - n_aug_;
    
    ///* the current NIS for radar
    //double NIS_radar_;
    
    ///* the current NIS for laser
    //double NIS_laser_;
    
    //create vector for weights
    weights_ = VectorXd(2*n_aug_+1);
    double weight_0 = lambda_/(lambda_ + n_aug_);
    
    weights_(0) = weight_0;
    for (int i=1; i<(2*n_aug_+1); i++) //2n+1 weights
    {
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage& meas_package)
{
    /**
     TODO:
     
     Complete this function! Make sure you switch between lidar and radar
     measurements.
     */
    
    if(is_initialized_)
    {
        const double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
        
        // First run prediction
        Prediction(delta_t);
        
        // Update measurement
        if((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_)
        {
            
        }
        else if(use_radar_)
        {
            // Assertion if different measurement appears
            assert(meas_package.sensor_type_ != MeasurementPackage::RADAR);
        }
    }
    else
    {
        is_initialized_ = true;
        
    }
    
    // Update timestamp
    time_us_ = meas_package.timestamp_;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(const double delta_t)
{
    /**
     TODO:
     
     Complete this function! Estimate the object's location. Modify the state
     vector, x_. Predict sigma points, the state, and the state covariance matrix.
     */
    
    // Generate sigma points
    // Predict sigma points
    // Predict mean and Covariance
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage& meas_package, const double delta_t)
{
    /**
     TODO:
     
     Complete this function! Use lidar data to update the belief about the object's
     position. Modify the state vector, x_, and covariance, P_.
     
     You'll also need to calculate the lidar NIS.
     */
    
    // Predict measurement
    
    // Update state
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage& meas_package, const double delta_t)
{
    /**
     TODO:
     
     Complete this function! Use radar data to update the belief about the object's
     position. Modify the state vector, x_, and covariance, P_.
     
     You'll also need to calculate the radar NIS.
     */
    
    // Predict measurement
    
    // Update state
}
