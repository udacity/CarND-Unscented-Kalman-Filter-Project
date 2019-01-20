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
  weights_ = VectorXd::Constant(2 * n_aug_ + 1, 0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  //cout << "weights_:\n"<< weights_ << endl;
  x_ << 1, 1, 1, 1, 0.1;
  P_ << 
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1;
  // cout<<"Initialize Done in ukf.cpp"<<endl;
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
      cout<<"First init is laser: "<<x_<<endl;
    }
    
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // transform radar (ro, theta, ro_dot) into x, y coordinate
      double ro = meas_package.raw_measurements_(0);
      double theta = meas_package.raw_measurements_(1);
      double ro_dot = meas_package.raw_measurements_(2);
      // Here I use the ro_dot as the v value.
      x_.head(3) << ro * cos(theta), ro*sin(theta), ro_dot;
      cout << "First init is radar: " << x_ << endl;
    }
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;

    // cout<<"Initialized in ProcessMeasurement done : "<<is_initialized_<<endl;
    return;
  }
  // Prediction step
  cout<<"Prediction start"<<endl;
  Prediction((meas_package.timestamp_ - time_us_)/1000000.0);
  cout<<"Prediction finished"<<endl;
  time_us_ = meas_package.timestamp_;
  // Update step
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    cout<<"Update lidar"<<endl;
    UpdateLidar(meas_package);
  }
  
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    cout<<"Update Radar"<<endl;
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
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
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
  lambda_ = 3 - n_aug_;
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++)
  { //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }
  // predict state mean
  x_ = (Xsig_pred_ * weights_.asDiagonal()).rowwise().sum();
  // predict state covariance matrix p
  P_ = (Xsig_pred_.colwise() - x_) * weights_.asDiagonal() *
      (Xsig_pred_.colwise() - x_).transpose();
  cout<<"Prediction x:\n"<<x_<<"\n Prediction p:\n"<<P_<<endl;
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
  cout<<"Zsig: \n"<<Zsig<<endl; // This line for the block assertion error.
  z_pred << (Zsig * weights_.asDiagonal()).rowwise().sum();
  cout<<"z_pred: \n"<<z_pred<<endl;
  // innovation covariance matrix S
  // for(int i =0; i<2*n_aug_ + 1; i++){
  //   VectorXd z_diff = Zsig.col(i) - z_pred;
  //   S = S + weights_(i) * z_diff * z_diff.transpose();
  // }
  // S += R;
  S << (Zsig.colwise() - z_pred)*weights_.asDiagonal()*(Zsig.colwise()-z_pred).transpose() + R;
  cout<<"S:\n"<<S<<endl;
  
  MatrixXd Tc(n_x_, n_z);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  cout<<"Tc :\n"<<Tc<<endl;
  // Tc = (Xsig_pred_.colwise() - x_) * weights_.asDiagonal() * (Zsig.colwise() - z_pred).transpose();
  // calculate Kalman gain K;
  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  cout<<"K: \n"<<K<<endl;
  VectorXd z(n_z);
  z << meas_package.raw_measurements_;
  cout<<"z:\n"<<z<<endl;
  // residual
  VectorXd z_diff = z - z_pred;
  cout<<"z_diff:\n"<<z_diff<<endl;
  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  cout << "x:\n"
       << x_ << endl;
  P_ = P_ - K * S * K.transpose();
  cout << "P:\n"
       << P_ << endl;
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
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  { // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                         // r
    Zsig(1, i) = atan2(p_y, p_x);                                     // phi
    Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); // r_dot
  }
  cout << "Zsig:\n"
       << Zsig << endl;
  // mean predicted measurement
  z_pred.fill(0.0);
  // calculate mean predicted measurement
  z_pred = (Zsig * weights_.asDiagonal()).rowwise().sum();
  // calculate innovation covariance matrix S
  for(int i=0; i< 2*n_aug_ + 1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while(z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while(z_diff(1) < -M_PI) z_diff(1)  += 2. * M_PI;
    S += S + weights_(i) * z_diff * z_diff.transpose();
  }
  // S = (Zsig.colwise() - z_pred) * weights_.asDiagonal() * (Zsig.colwise() - z_pred).transpose();
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;
  S = S + R;

  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_;
  // create matrix for cross correlation Tc
  MatrixXd Tc(n_x_, n_z);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  { // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  // Tc = (Xsig_pred_.colwise() - x_) * weights_.asDiagonal() * (Zsig.colwise() - z_pred).transpose();
  // calculate Kalman gain K;
  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1) > M_PI)
    z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI)
    z_diff(1) += 2. * M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  cout<<"Radar Update done\n x:\n"<<x_<<"\n P:\n"<<P_<<endl;
  // MatrixXd Tc = MatrixXd(n_x_, n_z);
  // // calculate cross correlation matrix
  // Tc = (Xsig_pred_.colwise() - x_) * weights_.asDiagonal() * (Zsig.colwise() - z_pred).transpose();
  // // calculate Kalman gain K;
  // MatrixXd K;
  // K = Tc * S.inverse();
  // // update state mean and covariance matrix
  // x_ += K * (z - z_pred);
  // P_ -= K * S * K.transpose();
  // cout<<"Radar Update Finished.\n"<<"x: \n"<<x_<<"P:\n"<<P_<<endl;
  // // TODO: calculate NIS
}