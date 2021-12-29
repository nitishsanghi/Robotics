#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

UKF::UKF() {
  //Initialized
  is_initialized_ = false;
    
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
    
    //State Covariance Matrix initialized
    P_ << 1,0,0,0,0,
          0,1,0,0,0,
          0,0,1,0,0,
          0,0,0,1,0,
          0,0,0,0,1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = .5;

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
  
  //Number of states initialized
  n_x_ = x_.size();
    
  //Number of augmentated states initialized
  n_aug_ = n_x_+ 2;
    
  //Lambda initialized
  lambda_ = 3 - n_aug_;
    
  //Augmentated State + Process Noise Covariance Matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);
    
  //std::cout << "Weights Calculated" << endl;
    VectorXd weights_ = VectorXd(2*n_aug_+1);
    
    
    
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    
    //Initializing if not initialized
    std::cout << "Entered ProcessMeasurement" << endl;
    if (!is_initialized_) {
        
        //First measurement used for initialization
        time_us_ = meas_package.timestamp_;
        cout << "UKF: " << endl;
        
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

           
            x_ << meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]), meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]),sqrt(meas_package.raw_measurements_[2]*cos(meas_package.raw_measurements_[1])*meas_package.raw_measurements_[2]*cos(meas_package.raw_measurements_[1]) + meas_package.raw_measurements_[2]*sin(meas_package.raw_measurements_[1])*meas_package.raw_measurements_[2]*sin(meas_package.raw_measurements_[1])),0,0;
        
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1],0, 0, 0;
            
        }
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    //Prediction of state and sigma points for Radar/Lidar
    double delta_t = (meas_package.timestamp_-time_us_)/1000000.0;
    time_us_ = meas_package.timestamp_;
    
    Prediction(delta_t);
    
    //Update of state based on measured and estimated using either Radar or Lidar functions
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

        UpdateRadar(meas_package);
        
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        
        UpdateLidar(meas_package);
    
    }
}

void UKF::Prediction(double delta_t) {
    
    //Creating Augmentation P Covariance Matrix
    MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
    
    P_aug_.topLeftCorner(n_x_,n_x_) << P_;
    
    P_aug_.topRightCorner(n_x_,(n_aug_-n_x_)) << 0,0,
                                                 0,0,
                                                 0,0,
                                                 0,0,
                                                 0,0;
    
    P_aug_.bottomLeftCorner((n_aug_-n_x_),n_x_) << 0,0,0,0,0,
                                                   0,0,0,0,0;
    
    P_aug_.bottomRightCorner((n_aug_-n_x_),(n_aug_-n_x_)) << std_a_*std_a_, 0,
                                                        0, std_yawdd_*std_yawdd_;
    
    //Finding Square root of Augmentation matrix term in sigma point generation term
    MatrixXd A = MatrixXd(n_aug_, n_aug_);
    A = P_aug_.llt().matrixL();
    MatrixXd B = MatrixXd(n_aug_,n_aug_);
    B << pow(3,.5)*A;
    
 
    //Augmented Sigma Point Matrix
    VectorXd x_aug_ = VectorXd(n_aug_);
    x_aug_.head(5) << x_;
    x_aug_.tail(2) << 0,
                      0;
    
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    for(int i = 0; i < 15; i++){
        if(i == 0){
            Xsig_aug.col(i) = x_aug_;
        }
        else if(0<i && i<8){
            Xsig_aug.col(i) = x_aug_ + B.col(i-1);
        }
        else{
            Xsig_aug.col(i) = x_aug_ - B.col(i-8);
        }
    }
    
 
    //Predicting Sigma Points
    for (int i = 0; i< 2*n_aug_+1; i++)
    {
        //Using code from lecture video
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;
        
        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;
        
        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }

    
 
    
    VectorXd weights = VectorXd(2*n_aug_+1);

    double weight_0 = lambda_/(lambda_+n_aug_);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {
        double weight = 0.5/(n_aug_+lambda_);
        weights(i) = weight;
        
    }
    
    
    //Predicting State
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_ = x_ + weights(i) * Xsig_pred_.col(i);
    }

    //Predicting State Covariance Matrix
    P_.fill(0.0);
    for(int j = 0; j < 2*n_aug_+1; j++){
        
        VectorXd  x_diff = Xsig_pred_.col(j)-x_;
        if (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        if (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        P_ = P_+ weights(j)*x_diff *x_diff.transpose();
    }
    
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
 
    VectorXd zm_ = meas_package.raw_measurements_;
    
    int n_z = 2;
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_+ 1);
    for(int i =0; i < 2 * n_aug_+ 1;i++){
        double px = (Xsig_pred_(0,i));
        double py = (Xsig_pred_(1,i));
        
        Zsig(0, i) = px;
        Zsig(1, i) = py;
    }
    
    VectorXd weights = VectorXd(2*n_aug_+1);
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {
        double weight = 0.5/(n_aug_+lambda_);
        weights(i) = weight;
        
    }
    
    
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    
    for (int i=0; i<2*n_aug_+1; i++) {
        z_pred = z_pred + weights(i)*Zsig.col(i);
    }
    
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    
    MatrixXd N = MatrixXd(2,2);
    N << std_laspx_*std_laspx_,0,
        0,std_laspy_*std_laspy_;
    
    for (int i=0; i<2*n_aug_+1; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S +weights(i)*(z_diff)*(z_diff).transpose();
    }
    S = S + N;
    
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    
    for(int i=0;i < 2*n_aug_+1;i++){
        Tc = Tc + weights(i)*(Xsig_pred_.col(i)-x_)*(Zsig.col(i)-z_pred).transpose();
    }
    
    //calculate Kalman gain K;
    MatrixXd K = MatrixXd(n_x_, n_z);
    K = Tc*S.inverse();
    
    //update state mean and covariance matrix
    x_ = x_ + K*(meas_package.raw_measurements_-z_pred);
    P_ = P_ - K*S*K.transpose();
    
    double e;
    
    e = (zm_-z_pred).transpose()*S.inverse()*(zm_-z_pred);
    std::cout << "NIS " << e << endl;
    
    std::cout << "State updated Lidar Exited" << endl;
 
}


void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //Prediction of Radar measurement
    
    //std::cout << "Measured Values Assigned" << endl;
    VectorXd zm_ = VectorXd(3);
    zm_ << meas_package.raw_measurements_[0],
           meas_package.raw_measurements_[1],
           meas_package.raw_measurements_[2];
   
        int n_z = 3;
        MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_+ 1);
        for(int i =0; i < 2 * n_aug_+ 1;i++){
            double px = (Xsig_pred_(0,i));
            double py = (Xsig_pred_(1,i));
            double v  = (Xsig_pred_(2,i));
            double an = (Xsig_pred_(3,i));
            double vx = v*cos(an);
            double vy = v*sin(an);
            double a = sqrt(px*px + py*py);
            double b = atan2(py,px);
            double c = (px*vx + py*vy)/a;
            
            Zsig.col(i) << a,
                           b,
                           c;
        }
    
    
    VectorXd weights = VectorXd(2*n_aug_+1);
    
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {
        double weight = 0.5/(n_aug_+lambda_);
        weights(i) = weight;
        
    }

        VectorXd z_pred = VectorXd(n_z);
        z_pred.fill(0.0);
        for (int i=0; i<2*n_aug_+1; i++) {
            z_pred = z_pred + weights(i)*Zsig.col(i);
        }

    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    
    MatrixXd N = MatrixXd(3,3);
    N << std_radr_*std_radr_,0,0,
         0,std_radphi_*std_radphi_,0,
         0,0,std_radrd_*std_radrd_;
    
    for (int i=0; i<2*n_aug_+1; i++) {
        VectorXd z_diff = Zsig.col(i)-z_pred;
        
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        
        S = S +weights(i)*z_diff*z_diff.transpose();
    }
    S = S + N;
    

    MatrixXd Tc = MatrixXd(n_x_, n_z);
    
    Tc.fill(0.0);
    
    for(int i=0;i < 2*n_aug_+1;i++){
        
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
        
        Tc = Tc + weights(i)*x_diff*z_diff.transpose();
    }
    
    //calculate Kalman gain K;
    MatrixXd K = MatrixXd(n_x_, n_z);
    K = Tc*S.inverse();
    
    //update state mean and covariance matrix
    //residual
    VectorXd z_diff = zm_ - z_pred;
    
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    x_ = x_ + K*z_diff;
    P_ = P_ - K*S*K.transpose();
    
    double e;
    
    e = z_diff.transpose()*S.inverse()*z_diff;
    
    std::cout << "NIS " << e << endl;
    
    std::cout << "State updated Radar Exited" << endl;
}
