#include <iostream>
#include "ukf.h"
#include "tools.h"

using namespace std;

// Define all constant tuning parameters
const double UKF::C_STD_ACC = 2.2;
const double UKF::C_STD_YAWDD = 0.9;
const double UKF::C_STD_LASPX = 0.16;
const double UKF::C_STD_LASPY = 0.16;
const double UKF::C_STD_RADR = 0.3;
const double UKF::C_STD_RADPHI = 0.03;
const double UKF::C_STD_RADRD = 0.3;
//

UKF::UKF():
    use_laser_(true),
    use_radar_(true),
    n_x_(5),
    n_aug_(7),
    n_z_radar_(3),
    n_z_lidar_(2),
    lambda_(3.0 - n_aug_),
    std_a_sq_(pow(C_STD_ACC, 2)),
    std_yawdd_sq_(pow(C_STD_YAWDD, 2)),
    std_laspx_sq_(pow(C_STD_LASPX, 2)),
    std_laspy_sq_(pow(C_STD_LASPY, 2)),
    std_radr_sq_(pow(C_STD_RADR, 2)),
    std_radphi_sq_(pow(C_STD_RADPHI, 2)),
    std_radrd_sq_(pow(C_STD_RADRD, 2)),
    weights_(Tools::calculateWeights(n_aug_, lambda_)),
    R_Radar_(getRadarMeasurementNoise()),
    R_Lidar_(getLidarMeasurementNoise())
{
    // State of the filter
    is_initialized_ = false;
    
    // Previous timestamp
    time_us_ = 0;
    
    // Initial state vector
    x_ = VectorXd(n_x_);
    
    // Initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);

    // Predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
}

MatrixXd UKF::getRadarMeasurementNoise() const
{
    // Set radar measurement noise
    MatrixXd R = MatrixXd(n_z_radar_, n_z_radar_);
    
    R <<
    std_radr_sq_, 0.,                      0.,
    0.,                  std_radphi_sq_, 0.,
    0.,                  0.,                      std_radrd_sq_;
    
    return R;
}

MatrixXd UKF::getLidarMeasurementNoise() const
{
    // Set Lidar measurement noise
    MatrixXd R = MatrixXd(n_z_lidar_, n_z_lidar_);
    
    R <<
    std_laspx_sq_, 0.,
    0., std_laspy_sq_;
    
    return R;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    // Check if UKF is initialized
    if(is_initialized_ && (use_laser_ || use_radar_))
    {
        //compute the time elapsed between the current and previous measurements
        double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;     //dt - expressed in seconds
        
        // Run small kalman filter preditiction steps if time step is too large
        while (dt > 0.1)
        {
            Prediction(0.1);
            dt -= 0.1;
        }
        
        // Run preditiction
        Prediction(dt);
        
        // Choose update method based on sensor type
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            UpdateRadar(meas_package);
        }
        else
        {
            UpdateLidar(meas_package);
        }
        
        // Update timestamp
        time_us_ = meas_package.timestamp_;
    }
    else
    {
        // Initialize the state x_ with the first measurement.
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            // Convert radar from polar to cartesian coordinates
            const double ro = meas_package.raw_measurements_(0);
            const double phi = meas_package.raw_measurements_(1);
            x_ <<
            ro * cos(phi),
            ro * sin(phi),
            0.0,
            0.0,
            0.0;
            
            // Initialize state covariance matrix
            P_ <<
            std_radr_sq_, 0.,                    0.,            0.,            0.,
            0.,                  std_radr_sq_, 0.,            0.,            0.,
            0.,                  0.,                    std_a_sq_,           0.,            0.,
            0.,                  0.,                    0.,            std_radphi_sq_,           0.,
            0.,                  0.,                    0.,            0.,            1.;
            
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            // Set lidar positions directly
            x_ <<
            meas_package.raw_measurements_(0),
            meas_package.raw_measurements_(1),
            0.0,
            0.0,
            0.0;
            
            // Initialize state covariance matrix
            P_ << std_laspx_sq_, 0.,                    0.,            0.,            0.,
            0.,                  std_laspy_sq_, 0.,            0.,            0.,
            0.,                  0.,                    0.1,           0.,            0.,
            0.,                  0.,                    0.,            0.1,           0.,
            0.,                  0.,                    0.,            0.,            0.1;
        }
        
        // Update timestamp and set initialized flag
        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
    }
}

void UKF::Prediction(const double delta_t)
{
    // Predict sigma points
    Xsig_pred_ = SigmaPointPrediction(delta_t);
    
    // Predict mean and covariance
    PredictMeanAndCovariance(&x_, &P_);
}


void UKF::UpdateLidar(const MeasurementPackage meas_package)
{
    // Lidar measurement of dimension 2: px, py
    const int n_z = n_z_lidar_;
    
    // Init variables
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd S = MatrixXd(n_z,n_z);
    
    // Predict lidar measurement in augmented state space
    PredictLidarMeasurement(&Zsig, &z_pred, &S);
    
    // Update state with current measurement
    const VectorXd z = meas_package.raw_measurements_;
    UpdateState(Zsig, z_pred, S, z, n_z);
    
    // Calculate NIS
    const VectorXd z_diff = z - z_pred;
    NIS_laser_ = Tools::calculateNIS(z_diff, S);
}

void UKF::UpdateRadar(const MeasurementPackage meas_package)
{
    // Radar measurement of dimension 3: rho, phi, rhodot
    const int n_z = n_z_radar_;
    
    // Init variables
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd S = MatrixXd(n_z,n_z);
    
    // Predict radar measurement in augmented state space
    PredictRadarMeasurement(&Zsig, &z_pred, &S);
    
    // Update state with current measurement
    VectorXd z = meas_package.raw_measurements_;
    z(1) = Tools::NormalizeAngle(z(1));
    UpdateState(Zsig, z_pred, S, z, n_z);
    
    // Calculate NIS
    VectorXd z_diff = z - z_pred;
    z_diff(1) = Tools::NormalizeAngle(z_diff(1));
    NIS_radar_ = Tools::calculateNIS(z_diff, S);
}

MatrixXd UKF::AugmentedSigmaPoints() const
{
    // Create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    
    // Create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    // Create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    // Create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0.;
    x_aug(6) = 0.;
    
    // Create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(5, 5) = std_a_sq_;
    P_aug(6, 6) = std_yawdd_sq_;
    
    // Create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    // Create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    
    for (int i = 0; i < n_aug_; i++)
    {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }
    
    return Xsig_aug;
}

MatrixXd UKF::SigmaPointPrediction(const double delta_t) const
{
    // Calculate augmented sigma points vectors
    const MatrixXd Xsig_aug = AugmentedSigmaPoints();
    
    // Create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    const double dtsq = delta_t*delta_t;
    
    // Predict sigma points
    for (int i=0; i<2*n_aug_+1; i++)
    {
        VectorXd x_k = VectorXd(n_aug_);
        x_k = Xsig_aug.col(i);
        double px = x_k(0);
        double py = x_k(1);
        double v = x_k(2);
        double psi = x_k(3);
        double psidot = x_k(4);
        double nu_a = x_k(5);
        double nu_psidot = x_k(6);
        
        VectorXd x_k1 = VectorXd(n_x_);
        
        // Avoid division by zero
        if (abs(psidot)>1e-3)
        {
            x_k1(0) = px + v/psidot * ( sin(psi+psidot*delta_t)  - sin(psi) ) ;
            x_k1(1) = py + v/psidot * ( -cos(psi+psidot*delta_t) + cos(psi) );
        }
        else
        {
            // No change in yaw, going on straight line
            x_k1(0) = px + v * delta_t * cos(psi);
            x_k1(1) = py + v * delta_t * sin(psi);
        }
        
        // Add noise
        x_k1(0) = x_k1(0) + 0.5*dtsq*cos(psi)*nu_a;
        x_k1(1) = x_k1(1) + 0.5*dtsq*sin(psi)*nu_a;
        
        x_k1(2) = v + 0 + delta_t*nu_a;
        x_k1(3) = psi + psidot*delta_t + 0.5*dtsq*nu_psidot;
        x_k1(4) = psidot + 0 + delta_t*nu_psidot;
        
        Xsig_pred.col(i) = x_k1;
    }
    
    return Xsig_pred;
}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) const
{
    // Short access to vector for predicted state
    VectorXd& x = *x_out;
    
    // Short access to covariance matrix for prediction
    MatrixXd& P = *P_out;
    
    // Predict state mean
    x.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++)
    {
        x += weights_(i)*Xsig_pred_.col(i);
    }
    
    // Predict state covariance matrix to mitigate non-positive-semi-definite issues
    P.fill(0.0);
    for (int i=1; i<2*n_aug_+1; i++)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - Xsig_pred_.col(0);
        x_diff(3) = Tools::NormalizeAngle(x_diff(3));
        
        P += weights_(i)*(x_diff)*(x_diff.transpose());
    }
}

void UKF::PredictLidarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out) const
{
    // Short access to measurement matrix
    MatrixXd& Zsig = *Zsig_out;
    
    // Transform sigma points into LIDAR measurement space
    for (int i=0; i<2*n_aug_+1; i++)
    {
        const double px = Xsig_pred_(0, i);
        const double py = Xsig_pred_(1, i);
        
        // Calculate rho
        Zsig(0,i) = px;
        Zsig(1,i) = py;
    }
    
    // Predict measurement
    PredictMeasurement(z_out, S_out, Zsig, R_Lidar_, n_z_lidar_);
}

void UKF::PredictRadarMeasurement(MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out) const
{
    // Transform sigma points into RADAR measurement space
    MatrixXd& Zsig = *Zsig_out;
    
    // Transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        // extract values for better readibility
        const double p_x = Xsig_pred_(0,i);
        const double p_y = Xsig_pred_(1,i);
        const double v  = Xsig_pred_(2,i);
        const double yaw = Xsig_pred_(3,i);
        
        const double v1 = cos(yaw)*v;
        const double v2 = sin(yaw)*v;
        
        const double rho =sqrt(p_x*p_x + p_y*p_y);
        
        // Set movement in measurement model
        Zsig(0,i) = rho;
        
        // Assume object going 90° if p_x is almost zero
        if(abs(p_x) > 1e-5)
        {
            Zsig(1,i) = atan2(p_y,p_x);
        }
        else
        {
            Zsig(1,i) = M_PI/2.;
        }
        
        // Set velocity to zero if rho movement is very small
        if(abs(rho) < 1e-5)
        {
            Zsig(2,i) = 0.0;
        }
        else
        {
            Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);
        }
        
    }
    
    // Predict measurement
    PredictMeasurement(z_out, S_out, Zsig, R_Radar_, n_z_radar_);
}

void UKF::PredictMeasurement(VectorXd* z_out, MatrixXd* S_out, const MatrixXd &Zsig, const MatrixXd &R, const int n_z) const
{
    // Calculate mean predicted measurement
    VectorXd& z_pred = *z_out;
    z_pred.fill(0.0);
    
    for (int i=0; i<2*n_aug_+1; i++)
    {
        z_pred += weights_(i)*Zsig.col(i);
    }
    
    // Calculate measurement covariance matrix S
    MatrixXd& S = *S_out;
    
    S.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++)
    {
        // State difference
        VectorXd z_diff = Zsig.col(i) - z_pred;
        if (n_z == 3)
            z_diff(1) = Tools::NormalizeAngle(z_diff(1));
        
        S += weights_(i)*(z_diff)*(z_diff.transpose());
    }
    // Add measurement noise
    S = S + R;
}

void UKF::UpdateState(const MatrixXd &Zsig, const VectorXd &z_pred, const MatrixXd &S, const VectorXd &z, const int n_z)
{
    // Matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    
    // Calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i=0; i<2*n_aug_+1; i++)
    {
        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        // Angle normalization
        x_diff(3) = Tools::NormalizeAngle(x_diff(3));
        
        // Measurement difference
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        // Angle normalization
        if (n_z == n_z_radar_)
            z_diff(1) = Tools::NormalizeAngle(z_diff(1));
        
        Tc += weights_(i)*(x_diff)*(z_diff.transpose());
    }
    
    // Calculate Kalman gain K
    MatrixXd K = MatrixXd(n_x_, n_z);
    K = Tc*(S.inverse());
    
    // Residual
    VectorXd z_diff = z - z_pred;
    if (n_z == 3)
        z_diff(1) = Tools::NormalizeAngle(z_diff(1));
    
    // Update state mean and covariance matrix
    x_ = x_ + K*z_diff;
    x_(3) = Tools::NormalizeAngle(x_(3));
    P_ = P_ - K*S*(K.transpose());
}
