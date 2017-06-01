#ifndef UNSCENTEDKF_FUSIONUKF_H
#define UNSCENTEDKF_FUSIONUKF_H

#include "measurement_package.h"
#include "tools.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionUKF {
public:

  bool is_initalized_;

  VectorXd x_;

  MatrixXd P_;

  MatrixXd Xsig_pred_;

  // Cache intermediate results
  MatrixXd x_pred;

  MatrixXd P_pred;

  MatrixXd X_diff;

  long long time_us_;

  double_t std_a_;

  double_t std_yawdd_;

  double_t std_laspx_;

  double_t std_laspy_;

  double_t std_radr_;

  double_t std_radphi_;

  double_t std_radrd_;

  VectorXd weights_;

  int n_x_;

  int n_aug_;

  int n_z_;

  double_t lambda_;

  /**
   * Constructor
   */
  FusionUKF();

  /**
   * Destructor
   */
  virtual ~FusionUKF();

  void UpdateRadar(MeasurementPackage meas_package);

  void Init(MeasurementPackage meas_package);

  MatrixXd _GenerateSigmaPoints();

  /**
   *
   * @param Xsig_aug
   */
  void _MotionPrediction(MatrixXd &Xsig_aug, double_t delta_t);

  MatrixXd _PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out,
                                 int norm_dim, MatrixXd &SIG);

  void _PropagateNoise(MatrixXd *S);

  MatrixXd _GetCrossCovariance(MatrixXd &X_diff, MatrixXd &Z_diff);
private:
  Tools tools;

  VectorXd _GenerateWeights(int dim);

  void _InitState(MeasurementPackage meas_package);

  void _InitProcessMatrix();


};


#endif //UNSCENTEDKF_FUSIONUKF_H
