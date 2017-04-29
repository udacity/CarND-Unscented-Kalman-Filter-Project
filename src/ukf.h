#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"
#include "OnlineStats.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
  bool is_initialized_; ///* initially set to false, set to true in first call of ProcessMeasurement
  bool use_laser_;      ///* if this is false, laser measurements will be ignored (except for init)
  bool use_radar_;      ///* if this is false, radar measurements will be ignored (except for init)
  int n_x_;             ///* State dimension
  int n_aug_;           ///* Augmented state dimension
  VectorXd x_;          ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  MatrixXd P_;          ///* state covariance matrix
  MatrixXd Xsig_pred_;  ///* predicted sigma points matrix
  long long time_us_;   ///* time when the state is true, in us
  double std_a_;        ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_yawdd_;    ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_laspx_;    ///* Laser measurement noise standard deviation position1 in m
  double std_laspy_;    ///* Laser measurement noise standard deviation position2 in m
  double std_radr_;     ///* Radar measurement noise standard deviation radius in m
  double std_radphi_;   ///* Radar measurement noise standard deviation angle in rad
  double std_radrd_ ;   ///* Radar measurement noise standard deviation radius change in m/s
  VectorXd weights_;    ///* Weights of sigma points
  double lambda_;       ///* Sigma point spreading parameter
  double NIS_radar_;    ///* the current NIS for radar
  double NIS_laser_;    ///* the current NIS for laser

  UKF();
  UKF(bool verboseMode, double std_a, double std_yawdd, bool dynamicProcesNoise, bool useLaser, bool useRadar);
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage& meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(const double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(const MeasurementPackage& meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(const MeasurementPackage& meas_package);

private:
  static constexpr int n_z_ = 3;
  bool verboseMode_;
  bool dynamicProcesNoise_;  // if true: update project noise params (accelaration & yaw_rate change) dynamically
  MatrixXd I;          // identity matrix
  MatrixXd H_laser_;   // measurement matrix - laser
  MatrixXd Ht_laser_;  // transpose of measurement matrix - laser
  MatrixXd R_laser_;   // measurement covariance matrix

  OnlineStats accelaration_stats_; // for dynamic stdev of acceleration
  OnlineStats yaw_dd_stats_;       // for dynamic stdev of yaw_rate change
  double  v0_;        // previous velocity
  double  yaw_d0_;    // previous yaw_rate

  void Initialize(const MeasurementPackage& meas_package);
  void GenerateSigmaPoints(); // not used
  void AugmentedSigmaPoints(MatrixXd& Xsig_aug, MatrixXd& P_aug);
  void SigmaPointPrediction(const MatrixXd& Xsig_aug, const MatrixXd& P_aug, double delta_t);
  void PredictMeanAndCovariance();

  void PredictRadarMeasurement(MatrixXd& Zsig, VectorXd& z_out, MatrixXd& S_out);
  void UpdateState(const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S, const VectorXd& z);
};

#endif /* UKF_H */
