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
  virtual ~UKF();

  void UpdateRadar(MeasurementPackage meas_package);

  void Prediction();

private:
  Tools tools;

  VectorXd _GenerateWeights(int dim);

  void _InitState(MeasurementPackage meas_package);

  void _InitProcessMatrix();

  MatrixXd _GenerateSigmaPoints();

  /**
   *
   * @param Xsig_aug
   */
  void _MotionPrediction(MatrixXd &Xsig_aug, double_t delta_t);

  void _PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out);

  MatrixXd Cart2Polar(const MatrixXd &Xsig);
};


#endif //UNSCENTEDKF_FUSIONUKF_H
