#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
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
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ = false;
  time_us_ = 0;
  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  lambda_ = 3 - n_x_;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  weights_ = VectorXd(2 * n_aug_ + 1);
  x_ << 1, 1, 1, 1, 0.1;
  P_ << 
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1;
}

// Descontructor.
UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   * ProcessMeasurement() function gets called in main.cpp. 
   * The main.cpp code contains a for loop that iterates through the data file one line 
   * at a time. For each line in the data file, ProcessMeasurement() gets called sending 
   * the sensor data to ukf.cpp
   */
  // Init step
  if(!is_initialized_){
    // is_initialized value init value is false
    // then init the value bsed on sensor
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // set px, and py to x state.
      x_.head(2) = meas_package.raw_measurements_;
    }
    
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // transform radar (ro, theta, ro_dot) into x, y coordinate
      double ro = meas_package.raw_measurements_(0);
      double theta = meas_package.raw_measurements_(1);
      double ro_dot = meas_package.raw_measurements_(2);
      // Here I use the ro_dot as the v value.
      x_.head(3) << ro * cos(theta), ro*sin(theta), ro_dot;
    }
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return;
  }
  // Prediction step
  Prediction((meas_package.timestamp_ - time_us_)/1000000.0);
  time_us_ = meas_package.timestamp_;
  // Update step
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  }
  else {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  // Generate Sigma Points
  n_x_ = 5;
  lambda_ = 3 - n_x_;
  MatrixXd Xsig(n_x_, 2*n_x_ + 1);
  MatrixXd A = P_.llt().matrixL();
  Xsig << x_,(sqrt(3)*A).colwise()+x_,(sqrt(3)* -A).colwise()+x_;
  
  // Augment Sigma Points
  VectorXd x_aug(n_aug_);
  MatrixXd Xsig_aug(n_aug_, 2*n_aug_+1);
  MatrixXd p_aug(n_aug_, n_aug_);
  x_aug << x_, 0, 0;
  p_aug.fill(0);
  p_aug.topLeftCorner(n_x_, n_x_) = P_;
  p_aug.bottomRightCorner(2, 2) << std_a_*std_a_, 0, 0, std_yawdd_ * std_yawdd_;
  MatrixXd L = p_aug.llt().matrixL();
  Xsig_aug << x_aug, (sqrt(3)*L).colwise()+x_aug, (sqrt(3)* -L).colwise()+x_aug;

  // Predict Sigma Points
  for(int i = 0; i < 2 * n_aug_; i++)
  {
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predict state value
    double px_p, py_p;
    if (fabs(yawd) > 0.001)
    {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else
    {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
  // predict state x and p
  // set weights
  weights_ = VectorXd::Constant(2 * n_aug_ + 1, 0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  // predict state mean
  x_ = (Xsig_pred_ * weights_.asDiagonal()).rowwise().sum();
  // predict state covariance matrix p
  P_ = (Xsig_pred_.colwise() - x_) * weights_.asDiagonal() *
      (Xsig_pred_.colwise() - x_).transpose();
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  int n_z = 2; //only px and py two values.
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z);
  MatrixXd R(2,2);
  R << pow(std_laspx_, 2), 0, 0, pow(std_laspy_, 2);

  // The value for lidar do not need to change.
  Zsig << Xsig_pred_.topRows(2);
  z_pred << (Zsig * weights_.asDiagonal()).colwise().sum();
  S << (Zsig.colwise() - z_pred)*weights_.asDiagonal()*(Zsig.colwise()-z_pred) + R;
  
  MatrixXd Tc(n_x_, n_z);
  Tc = (Xsig_pred_.colwise() - x_) * weights_.asDiagonal() * (Zsig.colwise() - z_pred).transpose();
  // calculate Kalman gain K;
  MatrixXd K;
  K = Tc * S.inverse();
  // update state mean and covariance matrix
  VectorXd z(n_z);
  z = meas_package.raw_measurements_;
  x_ += K * (z - z_pred);
  P_ -= K * S * K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  // transform sigma points into measurement space
  Zsig.row(0) = Xsig_pred_.topLeftCorner(2, 2 * n_aug_ + 1).colwise().norm();
  Zsig.row(1) = Xsig_pred_.row(1).array() / Xsig_pred_.row(0).array();
  // Zsig.row(1) = (Xsig_pred.row(1).array() / Xsig_pred.row(0).array()).atan();
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    Zsig.row(1)(i) = atan(Zsig.row(1)(i));
  }
  Zsig.row(2) = (Xsig_pred_.row(0).array() * Xsig_pred_.row(3).array().cos() +
                 Xsig_pred_.row(1).array() * Xsig_pred_.row(3).array().sin()) *
                Xsig_pred_.row(2).array() / Zsig.row(0).array();
  // calculate mean predicted measurement
  z_pred = (Zsig * weights_.asDiagonal()).rowwise().sum();
  // calculate innovation covariance matrix S
  S = (Zsig.colwise() - z_pred) * weights_.asDiagonal() * (Zsig.colwise() - z_pred).transpose();
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;
  S = S + R;

  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_;
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc = (Xsig_pred_.colwise() - x_) * weights_.asDiagonal() * (Zsig.colwise() - z_pred).transpose();
  // calculate Kalman gain K;
  MatrixXd K;
  K = Tc * S.inverse();
  // update state mean and covariance matrix
  x_ += K * (z - z_pred);
  P_ -= K * S * K.transpose();
  // TODO: calculate NIS
}