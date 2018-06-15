#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  std_a_ = 0.75;

  // Process noise standard deviation psi acceleration in rad/s^2
  std_yawdd_ = 0.3;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;  //-- DONT CHANGE - Given by manufacturer

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;  //-- DONT CHANGE - Given by manufacturer

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;  //-- DONT CHANGE - Given by manufacturer

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;  //-- DONT CHANGE - Given by manufacturer

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3; //-- DONT CHANGE - Given by manufacturer
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //set state dimension
  n_x_ = x_.size();

  //set augmented dimension
  n_aug_ = n_x_ + 2 ;   // n_sig_ sigma points

  // Number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //set vector for weights_
  weights_ = VectorXd(n_sig_);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  //create example matrix with predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // Measurement noise covariance matrices initialization
  R_radar_ = MatrixXd(3, 3);
  R_radar_ <<   std_radr_*std_radr_,      0,                          0,
                0,                      std_radphi_*std_radphi_,      0,
                0,                      0,                          std_radrd_*std_radrd_;

  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ <<   std_laspx_*std_laspx_,   0,
                0,                       std_laspy_*std_laspy_;

  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;

  // set to true in first call of ProcessMeasurement
  is_initialized_ = false;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization @ first measurement
   ****************************************************************************/

  if (!is_initialized_) {
    // first measurement
    cout << "Initialize UKF " << endl;

    ofstream out_data;
    out_data.open ("data_out.txt");

    x_.fill(0.0);

    P_.setIdentity(5, 5); // as discussed in Lesson 7, 32. What to Except from the Project, Initializing the State Covariance Matrix P_

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "First measurement: Radar" << endl;
       // Convert from polar to cartesian coordinates
      float rho     = meas_package.raw_measurements_[0];
      float phi     = meas_package.raw_measurements_[1];
      float rho_dot = meas_package.raw_measurements_[2];

      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      float v  = sqrt(vx * vx + vy * vy);

      // initialize state
      x_(0) = px;
      x_(1) = py;
      x_(3) = v;
    }

    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      cout << "First measurement: Lidar" << endl;
      // initialize state
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];

      if (fabs(x_(0)) < 0.0001 and fabs(x_(1)) <  0.0001){
            x_(0) =  0.0001;
            x_(1) =  0.0001;
      }
    }

    // initial timestamp for dt calculation
    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    cout << " Initialization finished" << endl;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
   //compute the time elapsed between the current and previous measurements
   double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt
   time_us_ = meas_package.timestamp_;

   Prediction(dt);

 /*****************************************************************************
  *  Update
  ****************************************************************************/
  ofstream out_data("data_out.txt", ios::app);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // Radar updates
    UpdateRadar(meas_package);
    float rho = meas_package.raw_measurements_(0);
    float phi = meas_package.raw_measurements_(1);
    out_data << rho * cos(phi) << ';';    //px - meas
    out_data << rho * sin(phi) << ';';    //py - meas
    cout << endl << "--------- Radar ---------" << endl;
  }

  else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ ) {
    // Laser updates
    UpdateLidar(meas_package);
    out_data << meas_package.raw_measurements_(0) << ';';    //px - meas
    out_data << meas_package.raw_measurements_(1) << ';';    //py - meas
    cout << endl << "--------- Lidar ---------" <<endl;
  }

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;

  out_data << NIS_radar_ << ';';
  out_data << NIS_laser_ << ';';
  out_data << x_(0) << ';';
  out_data << x_(1) << endl;


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} dt the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  ////////// Generate Sigma Points
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
  //create Q matrix
  MatrixXd Q = MatrixXd(n_aug_ - n_x_, n_aug_ - n_x_);

  // Setup vectors & matrices
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  Q <<  std_a_ * std_a_ ,   0,
        0,                  std_yawdd_ * std_yawdd_;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  //P_aug(5,5) = std_a_*std_a_;
//  P_aug(6,6) = std_yawdd_*std_yawdd_;
  P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q;

  //calculate square root of P_aug
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  //
  double sqrt_l_n_aug = sqrt(lambda_ + n_aug_);
  VectorXd sqrt_l_n_aug_A;

  for (int i = 0 ; i < n_aug_; i++){
    sqrt_l_n_aug_A = sqrt_l_n_aug * A.col(i);
    Xsig_aug.col(i+1)            = x_aug + sqrt_l_n_aug_A;
    Xsig_aug.col(i+1 + n_aug_)   = x_aug - sqrt_l_n_aug_A;
  }

  ////////// Predict Sigma Points
  double dt2 = dt*dt;

  for (int i = 0; i< n_sig_; i++){

      double px           = Xsig_aug (0,i);
      double py           = Xsig_aug (1,i);
      double v            = Xsig_aug (2,i);
      double psi          = Xsig_aug (3,i);
      double psi_dot      = Xsig_aug (4,i);
      double nu_a         = Xsig_aug (5,i);
      double nu_psi_dot   = Xsig_aug (6,i);

      double px_pre, py_pre, v_pre, psi_pre, psi_dot_pre;

      double sin_psi = sin(psi);
      double cos_psi = cos(psi);

      //avoid division by zero
      //                    <--state values   +    noise-->
      if(fabs(psi_dot) < 0.0001){
          px_pre = px + (v * cos_psi * dt)    +     0.5 * dt2 * cos_psi * nu_a;
          py_pre = py + (v * sin_psi * dt)    +     0.5 * dt2 * sin_psi * nu_a;
      }
      else{ //                                              <--state values   +    noise-->
          px_pre = px + (v/psi_dot) * ( sin(psi + psi_dot * dt) - sin_psi)    +    0.5 * dt2 * cos_psi * nu_a;
          py_pre = py + (v/psi_dot) * (-cos(psi + psi_dot * dt) + cos_psi)    +    0.5 * dt2 * sin_psi * nu_a;
      }

      v_pre           = v + dt * nu_a;
      psi_pre         = psi + psi_dot * dt + 0.5 * dt2 * nu_psi_dot;
      psi_dot_pre     = psi_dot + dt * nu_psi_dot;

      Xsig_pred_(0,i) = px_pre;
      Xsig_pred_(1,i) = py_pre;
      Xsig_pred_(2,i) = v_pre;
      Xsig_pred_(3,i) = psi_pre;
      Xsig_pred_(4,i) = psi_dot_pre;

  }
  ////////// Predict Mean and Covariance
  //predict state mean
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  //extract measurement as VectorXd
  VectorXd z = meas_package.raw_measurements_;

  //set measurement dimension, laser can measure px, py
  int n_z = 2; //// LASER

  //create matrix for sigma points in measurement space & transform sigma points into measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  for (int i = 0; i < n_sig_; i++) {
    Zsig(0,i) = Xsig_pred_(0,i); // px
    Zsig(1,i) = Xsig_pred_(1,i); // py;
  }

  Update_x_P(z, Zsig, n_z);

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //extract measurement as VectorXd
  VectorXd z = meas_package.raw_measurements_;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3; //// RADAR

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  //transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) {

    double px       = Xsig_pred_(0,i);
    double py       = Xsig_pred_(1,i);
    double v        = Xsig_pred_(2,i);
    double psi      = Xsig_pred_(3,i);

    double sqrt_pxpy = sqrt(px*px + py*py);

    Zsig(0,i) = sqrt_pxpy;      // rho
    Zsig(1,i) = atan2(py, px);               // psi
    Zsig(2,i) = (px * cos(psi)*v + py * sin(psi)*v) / (sqrt_pxpy);                 // rho_dot
  }

  Update_x_P(z, Zsig, n_z);

}


// Update function
void UKF::Update_x_P(VectorXd z, MatrixXd Zsig, int n_z){

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (z_diff(1)>  M_PI)      z_diff(1) = z_diff(1) - 2.*M_PI;
    while (z_diff(1)< -M_PI)      z_diff(1) = z_diff(1) + 2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();

  }

  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  if (n_z == 3){ // Radar
    S = S + R_radar_;
  }
  else if (n_z == 2){ // Lidar
    S = S + R_lidar_;
  }
  else {
    cout << "ERROR S = S + R";
  }

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  for (int i = 0; i < n_sig_; i++) {

    VectorXd z_diff = Zsig.col(i) - z_pred;
    if (n_z == 3) {
      //angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    }

   VectorXd x_diff = Xsig_pred_.col(i) - x_;
   //angle normalization
   while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
   while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

   Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;

  x_ = x_ + K * z_diff;

  P_ = P_ - K * S * K.transpose();

  if (n_z == 3) {// Radar
	   NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  }
  else if (n_z == 2) { // Lidar
	   NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  }

}
