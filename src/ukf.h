#ifndef UKF_H
#define UKF_H
#include "Eigen/Dense"
#include "measurement_package.h"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
    
    /**
     * Constructor
     */
    UKF();
    
    /**
     * Destructor
     */
    virtual ~UKF() { }
    
    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);
    
    /**
     * Get current state estimation
     * State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
     */
    VectorXd getStateEstimation() const
    {
        return x_;
    }
    
    /**
     * Get current 
     */
    double getNISLaser() const
    {
        return NIS_laser_;
    }
    
    /**
     *
     */
    double getNISRadar() const
    {
        return NIS_radar_;
    }
 
private:
    
    ///* if this is false, laser measurements will be ignored (except for init)
    const bool use_laser_;
    ///* if this is false, radar measurements will be ignored (except for init)
    const bool use_radar_;
    
    ///* State dimension
    const int n_x_;
    ///* Augmented state dimension
    const int n_aug_;
    
    const int n_z_radar_;
    
    const int n_z_lidar_;
    
    ///* Sigma point spreading parameter
    const double lambda_;
    
    ///* Process noise
    ///* standard deviation longitudinal acceleration in m/s^2
    static const double C_STD_ACC;
    const double std_a_sq_;
    
    ///* standard deviation yaw acceleration in rad/s^2
    static const double C_STD_YAWDD;
    const double std_yawdd_sq_;
    
    ///* Laser measurement noise
    ///* standard deviation position1 in m
    static const double C_STD_LASPX;
    const double std_laspx_sq_;
    
    ///* standard deviation position2 in m
    static const double C_STD_LASPY;
    const double std_laspy_sq_;
    
    ///* Radar measurement noise
    ///* standard deviation radius in m
    static const double C_STD_RADR;
    const double std_radr_sq_;
    
    ///* standard deviation angle in rad
    static const double C_STD_RADPHI;
    const double std_radphi_sq_;
    
    ///* standard deviation radius change in m/s
    static const double C_STD_RADRD;
    const double std_radrd_sq_;
    
    ///* Weights of sigma points
    const VectorXd weights_;
    
    const MatrixXd R_Radar_;
    
    const MatrixXd R_Lidar_;
    
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
    void UpdateLidar(const MeasurementPackage meas_package);
    
    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(const MeasurementPackage meas_package);
    
    /**
     *
     */
    MatrixXd AugmentedSigmaPoints() const;
    
    /**
     *
     */
    MatrixXd SigmaPointPrediction(const double delta_t) const;
    
    /**
     *
     */
    void PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) const;
    
    /**
     *
     */
    void PredictRadarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out, const int n_z) const;
    
    /**
     *
     */
    void PredictLidarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out, const int n_z) const;
    
    /**
     *
     */
    void PredictMeasurement(VectorXd* z_out, MatrixXd* S_out, const MatrixXd &Zsig, const MatrixXd &R, const int n_z) const;
    
    /**
     *
     */
    void UpdateState(const MatrixXd &Zsig, const VectorXd &z_pred, const MatrixXd &S, const VectorXd &z, const int n_z);
    
    /**
     *
     */
    MatrixXd getRadarMeasurementNoise() const;
    
    /**
     *
     */
    MatrixXd getLidarMeasurementNoise() const;
    
    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;
    
    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;
    
    ///* state covariance matrix
    MatrixXd P_;
    
    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;
    
    ///* time when the state is true, in us
    long time_us_;
    
    ///* the current NIS for radar
    double NIS_radar_;
    
    ///* the current NIS for laser
    double NIS_laser_;
    
};

#endif /* UKF_H */
