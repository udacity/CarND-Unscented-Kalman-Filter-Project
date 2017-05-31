#include "FusionUKF.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


FusionUKF::FusionUKF() {
  /**
   * Use augmented state
   */
  n_aug_ = 7;

  n_x_ = 5;

  lambda_ = 3;

  x_ = VectorXd::Zero(n_aug_);

  P_ = MatrixXd::Identity(n_aug_, n_aug_);

  Xsig_pred_ = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);

  std_a_ = 0.2;

  std_yawdd_ = 0.3;

  std_laspx_ = 0.15;

  std_laspy_ = 0.15;

  std_radr_ = 0.3;

  std_radphi_ = 0.03;

  std_radrd_ = 0.3;

  time_us_ = 0;
}

FusionUKF::~FusionUKF() {}


VectorXd FusionUKF::_GenerateWeights(int dim) {
  VectorXd weights = VectorXd::Zero(2 * dim + 1);
//  double_t weight_0 = lambda_/(lambda_);
  weights.fill(0.5/lambda_);
  weights(0) = 1;
  return weights;
}

void FusionUKF::_InitState(MeasurementPackage meas_package) {
  x_.fill(0.0);
  // x_(0) = 5.7441
  // x_(1) = 1.3800
  x_(0) = meas_package.raw_measurements_[0];
  x_(1) = meas_package.raw_measurements_[1];
  x_(2) = 2.2049;
  x_(3) = 0.5015;
  x_(4) = 0.3528;
  // x_(5) = 0;
  // x_(6) = 0;
}

void FusionUKF::_InitProcessMatrix() {
  MatrixXd P_sub = MatrixXd::Zero(n_x_, n_x_);

  P_sub << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
        -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
        0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
        -0.0022, 0.0071, 0.0007, 0.0098, 0.0010,
        -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

  P_.fill(0.0);
  P_.topLeftCorner(n_x_, n_x_) = P_sub;
  P_(5, 5) = pow(std_a_, 2);
  P_(6, 6) = pow(std_yawdd_, 2);
}

/**
 *
 * @return
 */
MatrixXd FusionUKF::_GenerateSigmaPoints() {
  MatrixXd L = P_.llt().matrixL();

  // create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

//  MatrixXd row_vector = MatrixXd::Ones(1, n_aug_);
//  Column duplication
  VectorXd row_vector = VectorXd::Ones(n_aug_);
  row_vector = row_vector.transpose();

  MatrixXd x_mat = x_ * row_vector;

  MatrixXd left_block = MatrixXd::Zero(n_aug_, n_aug_);
  left_block = x_mat + sqrt(lambda_) * L;

  MatrixXd right_block = MatrixXd::Zero(n_aug_, n_aug_);
  right_block = x_mat - sqrt(lambda_) * L;

  Xsig_aug.col(0) = x_;
  Xsig_aug.block(0, 1, n_aug_, n_aug_ + 1)  = left_block;
  Xsig_aug.block(0, n_aug_, n_aug_, 2 * n_aug_ + 1) = right_block;

  return Xsig_aug;
}

/**
 * Non linear mapping
 * @param Xsig_aug
 */
void FusionUKF::_MotionPrediction(MatrixXd &Xsig_aug, double_t delta_t){
  VectorXd p_x = Xsig_aug.row(0);
  VectorXd p_y = Xsig_aug.row(1);
  VectorXd v = Xsig_aug.row(2);
  VectorXd yaw = Xsig_aug.row(3);
  VectorXd yawd = Xsig_aug.row(4);
  VectorXd nu_a = Xsig_aug.row(5);
  VectorXd nu_yawdd = Xsig_aug.row(6);

  double_t threshold = 1e-3;
  int vec_len = 2 * n_aug_ + 1;
  VectorXd px_p = VectorXd::Zero(vec_len);
  VectorXd py_p = VectorXd::Zero(vec_len);
  for (int i = 0; i < vec_len; i+=1) {
    if(fabs(yawd(i)) > threshold) {
      px_p(i) = p_x(i) + v(i)/yawd(i) * ( sin(yaw(i) + yawd(i) * delta_t) - sin(yaw(i)));
      py_p(i) = p_y(i) + v(i)/yawd(i) * ( cos(yaw(i)) - cos(yawd(i) + yawd(i) * delta_t));
    } else {
      px_p(i) = p_x(i) + v(i) * delta_t * cos(yaw(i));
      py_p(i) = p_y(i) + v(i) * delta_t * sin(yaw(i));
    }
  }

  VectorXd v_p = v;
  // Element wise operations
  VectorXd yaw_p = yaw.array() + yaw.array() * delta_t;
  VectorXd yawd_p = yawd;

  // add noise
  px_p = px_p.array() + 0.5*nu_a.array()*pow(delta_t, 2)*cos(yaw.array());
  py_p = py_p.array() + 0.5*nu_a.array()*pow(delta_t, 2)*sin(yaw.array());
  v_p = v_p.array() + nu_a.array() * delta_t;

  yaw_p = yaw_p.array() + 0.5*nu_yawdd.array()*pow(delta_t, 2);
  yawd_p = yawd_p.array() + nu_yawdd.array()*delta_t;

  Xsig_pred_.row(0) = px_p;
  Xsig_pred_.row(1) = py_p;
  Xsig_pred_.row(2) = v_p;
  Xsig_pred_.row(3) = yaw_p;
  Xsig_pred_.row(4) = yawd_p;

  Xsig_pred_.row(5) = nu_a;
  Xsig_pred_.row(6) = nu_yawdd;
}
/**
 *
 * (Xsig_pred_ * w).tranpose()
 * @param x_out
 * @param P_out
 * @param norm_dim state/measurement vector dim needs to be normalized
 * @param SIG
 * State: Xsig_pred_
 * Measurement: Zsig
 * @return X - x_
 */
MatrixXd FusionUKF::_PredictMeanAndCovariance(VectorXd *x_out, MatrixXd *P_out,
                                          int norm_dim, MatrixXd &SIG) {
  VectorXd SIG_weights = SIG * weights_;
  VectorXd x = SIG_weights.transpose();
//  auto W?
  MatrixXd W = weights_.asDiagonal();
  // Column Duplication
  VectorXd row_vector = VectorXd::Ones(2 * n_aug_ + 1);
  row_vector = row_vector.transpose();

  MatrixXd x_mat = x * row_vector;
  MatrixXd X_diff = SIG - x_mat;

  // Normalization
  // For State  Xsig_pred 3: yaw
  // For Measurement Zsig 1: phi
  tools.NormalizeAngle(X_diff, norm_dim);
  MatrixXd P = X_diff * W * X_diff.transpose();

  *x_out = x;
  *P_out = P;
  return X_diff;
}

/**
 * Propagate additive measurement noise
 * @param S
 */
void FusionUKF::_PropagateNoise(MatrixXd *S) {
  MatrixXd R = MatrixXd::Zero(n_z_, n_z_);
  R(0, 0) = pow(std_radr_, 2);
  R(1, 1) = pow(std_radphi_, 2);
  R(2, 2) = pow(std_radrd_, 2);
  *S = *S + R;
}

void FusionUKF::_GetCrossCovariance(MatrixXd &X_diff, MatrixXd &Z_diff) {

}
