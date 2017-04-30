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
UKF::UKF() {
  cout<<"Initializing"<<endl;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  n_x_ = 5;

  n_aug_ = n_x_ + 2;

  n_sig_ = (2 * n_aug_) + 1;
  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

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
  P_ << 1,1,1,1,1,
        1,1,1,1,1,
        1,1,1,1,1,
        1,1,1,1,1,
        1,1,1,1,1;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;
  lambda_aug_ = 3 - n_aug_;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  cout<<"Process Measurements"<<endl;
  if(false == is_initialized_)    
  {
    cout<<"First measurement"<<endl;
    x_ = VectorXd(5);
    if( MeasurementPackage::LASER == meas_package.sensor_type_)
    {
      cout<<"First measurement is LASER"<<endl;
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      x_(2) = 0;
      x_(3) = 0;
      x_(4) = 0;
    }
    else
    {
      cout<<"First measurement is RADAR"<<endl;
      x_(0) = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
      x_(1) = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
      x_(2) = meas_package.raw_measurements_(2);
      x_(3) = meas_package.raw_measurements_(1);
      x_(4) = 0;
    }
    
    time_us_ = meas_package.timestamp_;    
    is_initialized_ = true;
    return;
  }

  double dt = (meas_package.timestamp_ - time_us_)/1000000;

  cout<<"Going to perform prediction"<<endl;
  Prediction(dt);

  if(MeasurementPackage::LASER == meas_package.sensor_type_)
  {
    cout<<"LASER update"<<endl;
    UpdateLidar(meas_package);
  }
  else
  {
    cout<<"RADAR update"<<endl;
    UpdateRadar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /* 1. Generate Sigma points */
  /* 2. Augment Sigma points with noise */
  cout<<"Calculating augmented sigma points"<<endl;
  MatrixXd x_aug = AugmentSigmaPoints();
  cout<<"Augmented X :"<<endl;
  cout<<x_aug;
  cout<<endl;
  /* 3. Augment Covariance Matrix with noise */
  /* 4. Predict state updated sigma points */
  cout<<"Predicting updated sigma points"<<endl;

  PredictSigmaPoints(x_aug, delta_t);
  cout<<Xsig_pred_;
  cout<<endl;
  /* 5. Get state vector and covariance from predicted sigma points */
  cout<<"Performing state update from predicted sigma points"<<endl;
  GetStateFromSigmaPoints();
  cout<<"State : "<<endl;
  cout<<x_;
  cout<<endl;
  cout<<"P Matrix : "<<endl;
  cout<<P_;
  cout<<endl;
}

MatrixXd UKF::GetSigmaPoints() {
  MatrixXd sigmaPoints(n_x_, (2 * n_x_) + 1);
  lambda_ = 3 - n_x_;

  MatrixXd sqrtP = P_.llt().matrixL();

  sqrtP = sqrt(lambda_ + n_x_) * sqrtP;

  sigmaPoints.col(0) = x_;
  for(int i = 1; i < n_x_ + 1; i++)
  {
    sigmaPoints.col(i) = x_ + sqrtP.col(i-1);
    sigmaPoints.col(i + n_x_) = x_ - sqrtP.col(i-1);
  }
  return sigmaPoints;
}

MatrixXd UKF::AugmentSigmaPoints() {
  MatrixXd augSigmaPoints(n_aug_, (2 * n_aug_) + 1);
  VectorXd x_aug(n_aug_);

  lambda_ = 3 - n_aug_;
  MatrixXd P_aug(n_aug_, n_aug_);
  MatrixXd Q(2,2);

  Q(0,0) = std_a_ * std_a_;
  Q(1,1) = std_yawdd_ * std_yawdd_;

  P_aug.fill(0.0);
  P_aug.block(0,0,n_x_,n_x_) = P_;
  P_aug.block(n_x_,n_x_,2,2) = Q;

  MatrixXd sqrtP = P_aug.llt().matrixL();
  sqrtP = sqrt(lambda_ + n_aug_) * sqrtP;

  x_aug.segment(0,n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_ + 1) = 0;

  augSigmaPoints.col(0) = x_aug;

  for(int i = 0; i < n_aug_; i++)
  {
    augSigmaPoints.col(i + 1) = x_aug + sqrtP.col(i);
    augSigmaPoints.col(i + n_aug_+ 1) = x_aug - sqrtP.col(i);
  }

  return augSigmaPoints;

}

void UKF::PredictSigmaPoints(MatrixXd x_aug, double time_elapsed) {

  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // State update Equations
    if(x_aug(4, i) != 0)
    {
      Xsig_pred_.col(i)(0) += x_aug(2 ,i) * ( sin(x_aug(3, i) + x_aug(4, i) * time_elapsed ) - sin(x_aug(3, i)) ) / x_aug(4, i);
      Xsig_pred_.col(i)(1) += x_aug(2, i) * ( -1 * cos(x_aug(3, i) + x_aug(4, i) * time_elapsed ) + cos(x_aug(3, i)) ) / x_aug(4, i);
      Xsig_pred_.col(i)(2) += 0;
      Xsig_pred_.col(i)(3) += x_aug(4, i) * time_elapsed;
      Xsig_pred_.col(i)(4) += 0;
    }
    else
    {
      Xsig_pred_.col(i)(0) += x_aug(2, i) * cos(x_aug(3, i));
      Xsig_pred_.col(i)(1) += x_aug(2, i) * sin(x_aug(3, i));
      Xsig_pred_.col(i)(2) += 0;
      Xsig_pred_.col(i)(3) += 0;
      Xsig_pred_.col(i)(4) += 0;
    }

    // Noise factor
    Xsig_pred_.col(i)(0) += 0.5 * (time_elapsed * time_elapsed) * cos(x_aug(3, i)) * x_aug(5, i);
    Xsig_pred_.col(i)(1) += 0.5 * (time_elapsed * time_elapsed) * sin(x_aug(3, i)) * x_aug(5, i);
    Xsig_pred_.col(i)(2) += time_elapsed * x_aug(5, i);
    Xsig_pred_.col(i)(3) += 0.5 * (time_elapsed * time_elapsed) * x_aug(6, i);
    Xsig_pred_.col(i)(4) += time_elapsed * x_aug(6, i);

  }
}

void UKF::GetStateFromSigmaPoints() {
  VectorXd x_k_1(n_aug_);

  x_k_1 = (lambda_/(lambda_ + n_aug_)) * Xsig_pred_.col(0);

  for(int i = 1; i < (2 * n_aug_ + 1); i++)
  {
    x_k_1 += ((0.5 * lambda_)/(lambda_ + n_aug_)) * Xsig_pred_.col(i);
  }

  MatrixXd P_k_1(n_aug_, n_aug_);

  P_k_1 = ((lambda_/lambda_ + n_aug_)) * ((Xsig_pred_.col(0) - x_k_1) * (Xsig_pred_.col(0) - x_k_1).transpose());
  for(int i = 1; i < (2 * n_aug_ + 1); i++)
  {
    P_k_1 += ((0.5 * lambda_)/(lambda_ + n_aug_)) * ((Xsig_pred_.col(0) - x_k_1) * (Xsig_pred_.col(0) - x_k_1).transpose());
  }

  x_ = x_k_1;
  P_ = P_k_1;
}

MatrixXd UKF::LidarGetZSigPoints(int n_z)
{
  MatrixXd Zsig(n_z, (2 * n_aug_) + 1);
  for(int i = 0; i < n_aug_; i++)
  {
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;

  }
  return Zsig;

}


VectorXd UKF::GetWeights()
{
  VectorXd weights = VectorXd(2*n_aug_ + 1);
  return weights;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
TODO:

Complete this function! Use lidar data to update the belief about the object's
position. Modify the state vector, x_, and covariance, P_.

You'll also need to calculate the lidar NIS.
*/
  int n_z = 2;
  VectorXd z_k_1_k(n_z);
  MatrixXd Zsig = LidarGetZSigPoints(n_z);
/*
  for(int i = 0; i < n_aug_; i++)
  {
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;

  }
  */
  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);

  double weight_0 = lambda_/(lambda_ + n_aug_);

  weights(0) = weight_0;

  for (int i=1; i<2*n_aug_+1; i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  cout<<"LIDAR : z_pred = "<<endl;
  cout<<z_pred;
  cout<<endl;

  cout<<"LIDAR : Zsig = "<<endl;
  cout<<Zsig;
  cout<<endl;
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    0.0225, 0, 
          0, 0.0225;

  S = S + R;

  cout<<"LIDAR : S = "<<endl;
  cout<<S;
  cout<<endl;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);


  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  cout<<"LIDAR : T = "<<endl;
  cout<<Tc;
  cout<<endl;
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  cout<<"LIDAR : K = "<<endl;
  cout<<K;
  cout<<endl;
  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  cout<<"LIDAR : z_diff = "<<endl;
  cout<<z_diff;
  cout<<endl;
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  cout<<"LIDAR : x after update : "<<endl;
  cout<<x_;
  cout<<endl;
  cout<<"LIDAR : P after update : "<<endl;
  cout<<P_;
  cout<<endl;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
TODO:

Complete this function! Use radar data to update the belief about the object's
position. Modify the state vector, x_, and covariance, P_.

You'll also need to calculate the radar NIS.
*/
  int n_z = 3;
  VectorXd z_k_1_k(n_z);
  MatrixXd Zsig(n_z, (2 * n_aug_) + 1);

  for(int i = 0; i < n_aug_; i++)
  {
    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    if(Zsig(0,i) < 0.00001)
    {
      Zsig(0,i) = 0.00001;
      Zsig(1,i) = 0;
    }
    else
    {
      Zsig(1,i) = atan2(p_y,p_x);                               //phi
    }
    Zsig(2,i) = (p_x*v1 + p_y*v2 )/Zsig(0,i);                   //r_dot

  }
  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);

  double weight_0 = lambda_/(lambda_ + n_aug_);

  weights(0) = weight_0;

  for (int i=1; i<2*n_aug_+1; i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  cout<<"RADAR : z_pred = "<<endl;
  cout<<z_pred;
  cout<<endl;

  cout<<"RADAR : Zsig = "<<endl;
  cout<<Zsig;
  cout<<endl;
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    0.09, 0, 0,
          0, 0.0009, 0,
          0, 0,0.09;


  S = S + R;

  cout<<"LIDAR : S = "<<endl;
  cout<<S;
  cout<<endl;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  cout<<"RADAR : T = "<<endl;
  cout<<Tc;
  cout<<endl;
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  cout<<"RADAR : K = "<<endl;
  cout<<K;
  cout<<endl;
  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  cout<<"RADAR : z_diff = "<<endl;
  cout<<z_diff;
  cout<<endl;
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  cout<<"RADAR : x after update : "<<endl;
  cout<<x_;
  cout<<endl;
  cout<<"RADAR : P after update : "<<endl;
  cout<<P_;
  cout<<endl;

}
