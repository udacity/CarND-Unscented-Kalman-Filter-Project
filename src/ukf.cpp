#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <iomanip>
#include <limits>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() : UKF(false, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()) {}
UKF::UKF(bool verboseMode, double std_a, double std_yawdd) : verboseMode_(verboseMode), is_initialized_(false), time_us_(0) {
  use_laser_ = true;    // if this is false, laser measurements will be ignored (except during init)
  use_radar_ = true;    // if this is false, radar measurements will be ignored (except during init)
  n_x_ = 5;             // State dimension
  n_aug_ = 2 + n_x_;    // Augmented state dimension
  lambda_ = 3 - n_aug_; // spread parameter
  x_ = VectorXd(n_x_);  // initial state vector
  P_ = MatrixXd(n_x_, n_x_);  // initial covariance matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);  // predicted sigma points matrix
  time_us_ = 0ll;       // time when the state is true, in us

  std_a_ = isnan(std_a) ? 0.4 : std_a;              // Process noise standard deviation longitudinal acceleration in m/s^2
  std_yawdd_ = isnan(std_yawdd) ? 0.3 : std_yawdd;  // Process noise standard deviation yaw acceleration in rad/s^2

  std_laspx_ = 0.15;    // Laser measurement noise standard deviation position1 in m
  std_laspy_ = 0.15;    // Laser measurement noise standard deviation position2 in m
  std_radr_ = 0.3;      // Radar measurement noise standard deviation radius in m
  std_radphi_ = 0.03;   // Radar measurement noise standard deviation angle in rad
  std_radrd_ = 0.3;     // Radar measurement noise standard deviation radius change in m/s
  lambda_ = 3 - n_x_;   // Sigma point spreading parameter
  NIS_radar_ = 0.0;     // the current NIS for radar
  NIS_laser_ = 0.0;     // the current NIS for laser

  Xsig_pred_.fill(0.0);
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = weights_(0) * 2 * lambda_;

  I = MatrixXd(n_x_, n_x_);
  I <<
    1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0;

  P_ << // I;
    std_laspx_, 0.0, 0.0, 0.0, 0.0,
    0.0, std_laspy_, 0.0, 0.0, 0.0,
    0.0, 0.0, std_radrd_, 0.0, 0.0,
    0.0, 0.0, 0.0, std_radphi_, 0.0,
    0.0, 0.0, 0.0, 0.0, std_radphi_;

  H_laser_ = MatrixXd(2, 5);
  H_laser_ << // measurement mapper - laser
    1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0;
  Ht_laser_ = H_laser_.transpose();

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << //measurement covariance matrix - laser
    std_laspx_*std_laspx_, 0.0,
    0.0, std_laspy_*std_laspy_;
}

UKF::~UKF() {}

void UKF::Initialize(const MeasurementPackage& meas_package) {
  // Initialize the state ekf_.x_ with the first measurement.
  if (verboseMode_) std::cerr << "EKF: " << fixed << setprecision(8) << endl;
  time_us_ = meas_package.timestamp_;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Convert radar from polar to cartesian coordinates and initialize state.
    auto rho = meas_package.raw_measurements_[0];
    auto phi = meas_package.raw_measurements_[1];
    auto rate = meas_package.raw_measurements_[2];
    x_ << rho*cos(phi), rho*sin(phi), rate, phi, 0;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0.0, 0.0, 0.0;
  }
  if (x_[0] == 0.0) x_[0] = 0.01; // init (to 0.01), to
  if (x_[1] == 0.0) x_[1] = 0.01; // overcome divide by zero
  is_initialized_ = true;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage& meas_package) {
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER && !use_laser_) return;
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && !use_radar_) return;
  if (!is_initialized_) {
    Initialize(meas_package);
    return;   // done initializing, no need to predict or update
  }

  auto delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);
  if (verboseMode_) {
    std::cerr << "PREDICTED:" << std::endl;
    std::cerr << "x_ = \t" << x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << x_(3) << "\t" << x_(4) << std::endl;
    std::cerr << "P_ = " << std::endl << P_ << std::endl;
  }

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else {
    UpdateLidar(meas_package);
  }

  // print the output
  if (verboseMode_) {
    std::cerr << "UPDATED:" << std::endl;
    std::cerr << "x_ = \t" << x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << x_(3) << "\t" << x_(4) << std::endl;
    std::cerr << "P_ = " << std::endl << P_ << std::endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(const double delta_t) {
  GenerateSigmaPoints();
  MatrixXd Xsig_aug; MatrixXd P_aug;
  AugmentSigmaPoints(Xsig_aug, P_aug);
  SigmaPointPrediction(Xsig_aug, P_aug, delta_t);
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage& meas_package) {
  // use standard Kalman filter for lidar measurement (which is linear), calculate NIS
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
  // update the state by using Kalman Filter equations
  VectorXd z_pred { H_laser_ * x_ };
  VectorXd y      { z - z_pred };
  MatrixXd PHt    { P_ * Ht_laser_ };
  MatrixXd S      { H_laser_ * PHt + R_laser_ };
  MatrixXd Si     { S.inverse() };
  MatrixXd K      { PHt * Si };

  // new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * H_laser_) * P_;
  NIS_laser_ = y.transpose()*Si*y;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage& meas_package) {
  // Use radar data
  MatrixXd Zsig; VectorXd z_pred; MatrixXd S;
  PredictRadarMeasurement(Zsig, z_pred, S);
  VectorXd z = VectorXd(3);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
  VectorXd e{ z - z_pred };
  NIS_radar_ = e.transpose()*S.inverse()*e;
  UpdateState(Zsig, z_pred, S, z);
}

void UKF::GenerateSigmaPoints() {
  auto A = MatrixXd{ P_.llt().matrixL() };
  Xsig_pred_.col(0) = x_;
  auto Pp = sqrt(lambda_ + n_x_) * A;
  for (auto i = 0; i < n_x_; ++i) {
    Xsig_pred_.col(i + 1) = x_ + Pp.col(i);
    Xsig_pred_.col(i + n_x_ + 1) = x_ - Pp.col(i);
  }
}

void UKF::AugmentSigmaPoints(MatrixXd& Xsig_aug, MatrixXd& P_aug) {
  //create augmented mean state
  auto x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_; x_aug(n_x_) = 0; x_aug(n_x_ + 1) = 0;

  //create augmented covariance matrix
  P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.row(n_x_).fill(0); P_aug.row(n_x_ + 1).fill(0);
  P_aug(n_x_, n_x_) = std_a_*std_a_; P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_*std_yawdd_;
  P_aug(n_x_, n_x_ + 1) = P_aug(n_x_ + 1, n_x_) = 0;

  //create sigma point matrix
  Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create square root matrix
  auto A = MatrixXd{ P_aug.llt().matrixL() };
  auto Pp = MatrixXd{ A * sqrt(lambda_ + n_aug_) };
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (auto i = 0; i<n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + Pp.col(i);
    Xsig_aug.col(i + n_aug_ + 1) = x_aug - Pp.col(i);
  }
}

void UKF::SigmaPointPrediction(const MatrixXd& Xsig_aug, const MatrixXd& P_aug, const double delta_t) {
  for (auto i = 0; i< 2 * n_aug_ + 1; ++i) {
    auto px = Xsig_aug(0, i), py = Xsig_aug(1, i);
    auto v = Xsig_aug(2, i), yaw = Xsig_aug(3, i), yawd = Xsig_aug(4, i);
    auto nu_a = Xsig_aug(5, i), nu_yawdd = Xsig_aug(6, i);

    auto yaw_inc = yawd*delta_t, half_dt2 = delta_t*delta_t / 2.0;
    auto cosyaw = cos(yaw), sinyaw = sin(yaw);

    auto is_yawd_z = (yawd>-0.001 && yawd<0.001);

    Xsig_pred_(0, i) = px + (is_yawd_z ? v*cosyaw*delta_t : v/yawd*(sin(yaw+yaw_inc)-sinyaw)) + half_dt2*cosyaw*nu_a;
    Xsig_pred_(1, i) = py + (is_yawd_z ? v*sinyaw*delta_t : v/yawd*(-cos(yaw+yaw_inc)+cosyaw)) + half_dt2*sinyaw*nu_a;
    Xsig_pred_(2, i) = v  + delta_t*nu_a;
    Xsig_pred_(3, i) = yaw  + yaw_inc + half_dt2*nu_yawdd;
    Xsig_pred_(4, i) = yawd + delta_t*nu_yawdd;
  }
}

void UKF::PredictMeanAndCovariance() {
  x_.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; ++i) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  //predict state covariance matrix
  P_.fill(0.0);
  for (auto i = 0; i<2 * n_aug_ + 1; ++i) {
    auto e = VectorXd{ Xsig_pred_.col(i) - x_ };
    Tools::NormalizeAngle(e(3));
    P_ += weights_(i) * (e*e.transpose());
  }
}

constexpr int UKF::n_z_; // define storage, needed, otherwise get link errors with gcc

void UKF::PredictRadarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S) {
  //transform sigma points into measurement space
  Zsig = MatrixXd(UKF::n_z_, 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  for (auto i = 0; i<2 * n_aug_ + 1; ++i) {
    double px = Xsig_pred_(0, i), py = Xsig_pred_(1, i), v = Xsig_pred_(2, i), yaw = Xsig_pred_(3, i);

    Zsig(0, i) = sqrt(px*px + py*py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px*cos(yaw)*v + py*sin(yaw)*v) / Zsig(0, i);
  }

  //mean predicted measurement
  z_pred = VectorXd(UKF::n_z_);
  z_pred.fill(0.0);
  for (auto i = 0; i<2 * n_aug_ + 1; ++i) {
    z_pred += weights_(i)*Zsig.col(i);
  }

  //measurement covariance matrix S
  S = MatrixXd(UKF::n_z_, UKF::n_z_);
  //calculate measurement covariance matrix S
  S.fill(0.0);
  S(0, 0) = std_radr_*std_radr_; S(1, 1) = std_radphi_*std_radphi_; S(2, 2) = std_radrd_*std_radrd_;
  for (auto i = 0; i<2 * n_aug_ + 1; ++i) {
    auto e = VectorXd{ Zsig.col(i) - z_pred };
    Tools::NormalizeAngle(e(1));
    S += weights_(i)*(e*e.transpose());
  }
}

void UKF::UpdateState(const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S, const VectorXd& z) {
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, UKF::n_z_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (auto i = 0; i<2 * n_aug_ + 1; ++i) {
    auto ez = VectorXd{ Zsig.col(i) - z_pred };
    Tools::NormalizeAngle(ez(1));

    auto ex = VectorXd{ Xsig_pred_.col(i) - x_ };
    Tools::NormalizeAngle(ex(3));

    Tc += weights_(i) * ex * ez.transpose();
  }

  //calculate Kalman gain K;
  auto K = MatrixXd{ Tc * S.inverse() };

  //update state mean and covariance matrix
  auto ez = VectorXd{ z - z_pred };
  Tools::NormalizeAngle(ez(1));
  x_ += K*ez;
  P_ -= K*S*K.transpose();
}
