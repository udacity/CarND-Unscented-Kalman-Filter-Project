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
        else if((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_)
        {
        }
    }
    else
    {
        // Check measurement type
        if(meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            // Init CTRV model with laser data.
            // Assume velocity, turn angle and turn rate to be zero
            x_ <<
            meas_package.raw_measurements_(0),
            meas_package.raw_measurements_(1),
            0.0,
            0.0,
            0.0;
            is_initialized_ = true;
        }
        else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            // Init CTRV model with radar data.
            // Assume velocity, turn angle and turn rate to be zero
            double ro = meas_package.raw_measurements_(0);
            double phi = meas_package.raw_measurements_(1);
            x_ <<
            ro * cos(phi),
            ro * sin(phi),
            0.0,
            0.0,
            0.0;
            is_initialized_ = true;
        }
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

MatrixXd UKF::getAugmentedSigmaPoints()
{
    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    /*******************************************************************************
     * Student part begin
     ******************************************************************************/
    
    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;
    
    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++)
    {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }
    
    return Xsig_aug;
}

MatrixXd UKF::SigmaPointPrediction(const double delta_t)
{
    
    MatrixXd Xsig_aug = getAugmentedSigmaPoints();
    
    //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    //predict sigma points
    for (int i = 0; i< 2*n_aug_+1; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > 0.001)
        {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else
        {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;
        
        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;
        
        //write predicted sigma point into right column
        Xsig_pred(0,i) = px_p;
        Xsig_pred(1,i) = py_p;
        Xsig_pred(2,i) = v_p;
        Xsig_pred(3,i) = yaw_p;
        Xsig_pred(4,i) = yawd_p;
    }
    
    
    return Xsig_pred;
}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out, const MatrixXd& Xsig_pred)
{
    //create vector for predicted state
    VectorXd x = VectorXd(n_x_);
    
    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x_, n_x_);
    
    //predicted state mean
    x.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x = x+ weights_(i) * Xsig_pred.col(i);
    }
    
    //predicted state covariance matrix
    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        
        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        P = P + weights_(i) * x_diff * x_diff.transpose() ;
    }
    
    *x_out = x;
    *P_out = P;
}
