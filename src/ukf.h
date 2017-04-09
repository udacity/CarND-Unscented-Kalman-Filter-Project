#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF
{
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
     * @return state vector [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
     */
    VectorXd getStateEstimation() const
    {
        return x_;
    }
    
    /**
     * Get current NIS (Normalized Innovation Squared) for latest laser measurement
     * @return NIS estimation
     */
    double getNISLaser() const
    {
        return NIS_laser_;
    }
    
    /**
     * Get current NIS (Normalized Innovation Squared) for latest radar measurement
     * @return NIS estimation
     */
    double getNISRadar() const
    {
        return NIS_radar_;
    }
 
private:
    
    ///* If this is false, laser measurements will be ignored (except for init)
    const bool use_laser_;
    
    ///* If this is false, radar measurements will be ignored (except for init)
    const bool use_radar_;
    
    ///* State dimension
    const int n_x_;
    
    ///* Augmented state dimension
    const int n_aug_;
    
    ///* Number of states in radar measurement
    const int n_z_radar_;
    
    ///* Number of states in lidar measurement
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
    
    ///* Measurement noise covariance matrix for lidar
    const MatrixXd R_Radar_;
    
    ///* Measurement noise covariance matrix for radar
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
     * Get augmented sigma points
     * @return augmented sigma points
     */
    MatrixXd AugmentedSigmaPoints() const;
    
    /**
     * Predict sigma points
     * @return predicted sigma points
     */
    MatrixXd SigmaPointPrediction(const double delta_t) const;
    
    /**
     * Predict mean and covariance matrix for current sigma point estimations
     * @param x_out Predicted state estimation (Output)
     * @param P_out Predicted covariance matrix (Output)
     */
    void PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) const;
    
    /**
     * Prediction step for radar measurement
     * @param Zsig_out Sigma points in measurement space (Output)
     * @param z_out Predicted measurement (Output)
     * @param S_out Covariance matrix S (Output)
     */
    void PredictRadarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out) const;
    
    /**
     * Prediction step for lidar measurement
     * @param Zsig_out Sigma points in measurement space (Output)
     * @param z_out Predicted measurement (Output)
     * @param S_out Covariance matrix S (Output)
     */
    void PredictLidarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out) const;
    
    /**
     * Common preditiction step for radar and lidar measurement
     * @param z_out Predicted measurement (Output)
     * @param S_out Covariance matrix S (Output)
     * @param Zsig Sigma points in measurement space
     * @param R Measurement noise covariance matrix
     * @param n_z Measurement dimension
     */
    void PredictMeasurement(VectorXd* z_out, MatrixXd* S_out, const MatrixXd &Zsig, const MatrixXd &R, const int n_z) const;
    
    /**
     * Update step of UKF algorithm
     * @param Zsig Sigma points in measurement space
     * @param z_pred Mean predicted measurement
     * @param S Predicted measurement covariance
     * @param z Incoming measurement
     * @param n_z Measurement dimension
     */
    void UpdateState(const MatrixXd &Zsig, const VectorXd &z_pred, const MatrixXd &S, const VectorXd &z, const int n_z);
    
    /**
     * Calculate lidar measurement noise from constants
     * @return measurement noise matrix
     */
    MatrixXd getRadarMeasurementNoise() const;
    
    /**
     * Calculate lidar measurement noise from constants
     * @return measurement noise matrix
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
