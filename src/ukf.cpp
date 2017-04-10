#include "ukf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(double std_radr, double std_radphi, double std_radrd) {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd::Zero(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ =  30/100;

  // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30/100;

  // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.009;

  // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.009;

  // Radar measurement noise standard deviation radius in m
    std_radr_ = std_radr;// 0.091;

  // Radar measurement noise standard deviation angle in rad
    std_radphi_ = std_radphi;// 0.015;

  // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = std_radrd; //.07;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5; // 5 state dimensions
  n_aug_ = n_x_ +2; // Augmented noise states
  Xsig_pred_ = MatrixXd::Zero(n_x_,2*n_aug_ + 1); // TODO: Dimensions
  nradar_z = 3;
  nlaser_z = 2;
  P_aug_ = MatrixXd::Zero(n_aug_, n_aug_);
  lambda_ = 3 - n_x_;
  weights_ = VectorXd::Zero(2*n_aug_+1);
  /* Covariance matrix */
  // TODO: Inital values?
  P_ = MatrixXd::Identity(n_x_, n_x_);

  Xsig_aug_ = MatrixXd::Zero(7,7*2+1);

  x_aug_ = VectorXd::Zero(n_x_ + 2);
  S_laser = MatrixXd(nlaser_z, nlaser_z);
  S_radar = MatrixXd(nradar_z, nradar_z);

}

inline double wrapAngle( double angle )
{
  if(fabs(angle) <= 0.001f)
  {
    return angle;
  }
  double twoPi = 2.0 * 3.141592865358979;
  return angle - twoPi * floor( angle / twoPi );
}

UKF::~UKF() {}
/**
 * @returns bool True if there were no errors false otherwise
 */
bool UKF::GenerateSigmaPoints(void)
{
  bool status = true;
  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();
  Xsig_pred_.col(0) << x_;
  double sq_lambda = sqrt(lambda_+n_x_);
  for(int j=1; j<n_x_+1; ++j)
  {
    for(int i=0; i<n_x_; ++i)
    {
      std::cout << x_[i] + sq_lambda * A(i,j-1) << '\n';
      Xsig_pred_(i,j) = x_[i] + sq_lambda * A(i,j-1);

    }
    //std::cout << "------" << '\n';
  }

  for(int j=1+n_x_; j<2*n_x_+1; ++j)
  {
    for(int i=0; i<n_x_; ++i)
    {
      std::cout << x_[i] - sq_lambda * A(i,j-(1+n_x_)) << '\n';
      Xsig_pred_(i,j) = x_[i] - sq_lambda * A(i,j-(1+n_x_));
    }
  }


  return (status);
}
/**
 * @returns bool True if there were no errors false otherwise
 */
bool UKF::AugmentedSigmaPoints(void)
{
  bool status = true;
  VectorXd noise = VectorXd::Zero(2);

  //create augmented mean state
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  MatrixXd p_aug = MatrixXd::Zero(n_aug_, n_aug_);
  x_aug.head(5) = x_;
  //x_aug(5) = 0;
  //x_aug(6) = 0;

  //create augmented covariance matrix
  p_aug.topLeftCorner(n_x_,n_x_) = P_;
  p_aug(5,5) = std_a_*std_a_;
  p_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = p_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  double_t sqrt_lambda_n_aug = sqrt(lambda_+n_aug_);
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug_.col(i+1)        = x_aug + sqrt_lambda_n_aug * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt_lambda_n_aug * L.col(i);
  }
  return (status);
}
/**
 * @returns bool True if there were no errors false otherwise
 */
bool UKF::SigmaPointPrediction(void)
{
  bool status = true;
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001)
    {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
    }
    else
    {
      px_p = p_x + v*dt*cos(yaw);
      py_p = p_y + v*dt*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*dt;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*dt*dt * cos(yaw);
    py_p = py_p + 0.5*nu_a*dt*dt * sin(yaw);
    v_p = v_p + nu_a*dt;

    yaw_p = yaw_p + 0.5*nu_yawdd*dt*dt;
    yawd_p = yawd_p + nu_yawdd*dt;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  std::cout << Xsig_pred_ << std::endl;
  std::cout << Xsig_aug_ << std::endl;
  return (status);
}
/**
 * @returns bool True if there were no errors false otherwise
 */
bool UKF::PredictMeanAndCovariance(void)
{
  bool status = true;

  //predict state mean TODO: Clean X and P ?
  VectorXd tmp_x = VectorXd::Zero(n_x_);
  MatrixXd tmp_P = MatrixXd::Zero(n_x_,n_x_);
  for (size_t i = 0; i < 2*n_aug_+1; i++) {
    tmp_x = tmp_x + weights_[i] * Xsig_pred_.col(i);
  }
  //predict state covariance matrix
  //VectorXd x_diff;
  for (size_t i = 0; i < 2*n_aug_+1  ; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - tmp_x;
    //angle normalization
    // set the angle between +Pi and -Pi
    x_diff(3) = wrapAngle(x_diff(3));
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    std::cout <<  "x_diff[3]:  ";
    std::cout << x_diff[3] << "  dt: " << dt << std::endl;



    tmp_P +=  weights_[i]*x_diff*x_diff.transpose();
  }
  x_ = tmp_x;
  P_ = tmp_P;

  return (status);
}
/**
 * @returns bool True if there were no errors false otherwise
 */
bool UKF::PredictRadarMeasurement(void)
{
  bool status = true;
  return (status);
}
/**
 * @returns bool True if there were no errors false otherwise
 */
bool UKF::UpdateState(void)
{
  bool status = true;
  return (status);
}

inline VectorXd state2MeasVect(VectorXd state)
{
  double rho;
  double phi;
  double rho_dot;
  rho = sqrt(state[0] *state[0] + state[1]*state[1] );
  phi = atan2(state[1] , state[0]);
  rho_dot = (state[0] * cos(state[3])*state[2] + \
                state[1] * sin(state[3]) * state[2]);
  if(fabs(rho) >= 0.001f)
  {
    rho_dot /= rho;
  }
  else
  {
    std::cout << ".";
    rho_dot /= 0.001f;
  }


  VectorXd ret_vec = VectorXd::Zero(3); // TODO: Este objeto sobrevive a la funcion?
  ret_vec <<  rho, phi, rho_dot;
  return (ret_vec);
}



/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if(!is_initialized_)
  {
    P_ = MatrixXd::Identity(n_x_, n_x_);
    GenerateSigmaPoints();
    /* Init weights_ */
    //set weights
    weights_[0]  = lambda_ / (lambda_ + n_aug_);
    /* Set w for each column i.e. each of the sigma points */
    for (size_t i = 1; i < 2*n_aug_+1; i++)
    {
      weights_[i] = 1 / (2*(lambda_ + n_aug_));
    }
    if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      if(fabs(meas_package.raw_measurements_[0]) >= 0.001f || \
          fabs(meas_package.raw_measurements_[0]) >= 0.001f ){
        x_ <<  meas_package.raw_measurements_[0],  meas_package.raw_measurements_[1], \
            0.0f, 0.0f;
      } else{
        x_ << 0.001, 0.001,0,0;
      }

      std::cout << x_ << std::endl;
    } else /* Measurement is radar */
    {
      //x_.fill(0.0f);
      if(fabs(meas_package.raw_measurements_[0]) >= 0.001f && \
          fabs(meas_package.raw_measurements_[1]) >= 0.001f && \
          fabs(meas_package.raw_measurements_[2]) >= 0.001f
          ) {

        double_t px = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
        double_t py = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
        if (fabs(px) <= 0.001f || fabs(py) <= 0.001f) {
          return;
        }
        x_ << px,py,meas_package.raw_measurements_[2],meas_package.raw_measurements_[1],0;

      }
      x_ << 0.001f, 0.001f, 0.00f,0,0;
    }
    cout << "X:" << x_;
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  dt = (meas_package.timestamp_ - previous_timestamp_)/ (double_t)1000000.0;
  if(dt > 0.1)
  {
    std::cout << "XX";
  }
  UKF::Prediction(dt);
  if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    UpdateLidar(meas_package);
  } else if(use_radar_) /* Is radar */
  {
    UpdateRadar(meas_package);
  }
  previous_timestamp_ = meas_package.timestamp_;
  cout << "X:" << x_;
  cout << "\n----" << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
 // GenerateSigmaPoints();
  AugmentedSigmaPoints();
  SigmaPointPrediction();
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  //
  MatrixXd Tc = MatrixXd::Zero(n_x_, nlaser_z);
  MatrixXd R  = MatrixXd::Zero(nlaser_z, nlaser_z);
  MatrixXd Zsig = MatrixXd::Zero(nlaser_z, 2*n_aug_+1);
  VectorXd z_pred = VectorXd::Zero(nlaser_z);
  //calculate cross correlation matrix
  S_laser.fill(0.0f);
  for (size_t i = 0; i < 2*n_aug_+1; i++)
  {
    Zsig(0, i) = Xsig_pred_(0, i); // Px
    Zsig(1, i) = Xsig_pred_(1, i); // Py
  }

  for (int j = 0; j < 2*n_aug_+1; ++j)
  {
   z_pred += weights_[j] * Zsig.col(j);
  }
  S_laser.fill(0.0f);
  for (int k = 0; k < 2*n_aug_+1; ++k) {
    //VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(k) - z_pred;
    S_laser += weights_(k) * z_diff * z_diff.transpose();
  }

  R(0,0) = std_laspx_*std_laspx_;
  R(1,1) = std_laspy_*std_laspy_;
  S_laser += R;

  for (int l = 0; l <2*n_aug_+1; ++l) {
    VectorXd x_diff = Xsig_pred_.col(l) - x_;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    x_diff(3)= wrapAngle(x_diff(3));
    VectorXd z_diff = Zsig.col(l) - z_pred;
    Tc += weights_[l] * x_diff * z_diff.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd Si = S_laser.inverse();
  MatrixXd Kg = Tc * Si;
  //update state mean and covariance matrix
  x_ = x_ + Kg*(meas_package.raw_measurements_ - z_pred);

  P_ = P_ - Kg*S_laser*Kg.transpose();

  NIS_laser_ = (meas_package.raw_measurements_ - z_pred).transpose() * Si * (meas_package.raw_measurements_ - z_pred);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  MatrixXd Si;
  MatrixXd R  = MatrixXd::Zero(nradar_z, nradar_z);
  MatrixXd Tc = MatrixXd::Zero(n_x_, nradar_z);
  MatrixXd Zsig = MatrixXd::Zero(nradar_z, 2*n_aug_+1);
  VectorXd z_pred = VectorXd::Zero(nradar_z);
  S_radar.fill(0.0f);
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;
  //calculate cross correlation matrix
  // Get the predicted to measurement space
  for (int j = 0; j <2*n_aug_+1; ++j)
  {// TODO: is this correctly done?
    VectorXd state = Xsig_pred_.col(j);
    //Zsig.col(j) = state2MeasVect(Xsig_pred_.col(j));
    double rho;
    double phi;
    double rho_dot;
    rho = sqrt(state[0] *state[0] + state[1]*state[1] );
    phi = atan2(state[1] , state[0]);
    rho_dot = (state[0] * (cos(state[3])*state[2]) +  state[1] * sin(state[3]) * state[2]);
    if(fabs(rho) >= 0.0001f)
    {
      rho_dot /= rho;
    }
    else
    {
      std::cout << ".";
      rho_dot /= 0.0001f;
    }
    Zsig.col(j) << rho, phi, rho_dot;
  }
  //z_pred.fill(0.0f);
  for (int k = 0; k < 2*n_aug_+1; k++)
  {
    z_pred += weights_[k] * Zsig.col(k);
  }
  S_radar.fill(0);
  for (size_t i = 0; i < 2*n_aug_+1; i++) {
    //VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = wrapAngle(z_diff(1));
   // while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    // while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_radar += weights_(i) * z_diff * z_diff.transpose();
  }
  S_radar += R;

  for (int i = 0; i < 2*n_aug_+1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = wrapAngle(z_diff(1));
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    x_diff(3)= wrapAngle(x_diff(3));
    Tc += weights_[i] * x_diff * z_diff.transpose();
  }
  //calculate Kalman gain K;
  Si = S_radar.inverse();
  MatrixXd Kg = Tc * Si;
  VectorXd z_diff_g = meas_package.raw_measurements_ - z_pred;
  //while (z_diff_g(1)> M_PI) z_diff_g(1)-=2.*M_PI;
  z_diff_g(1)= wrapAngle(z_diff_g(1));
  //update state mean and covariance matrix
  x_ = x_ + Kg*(z_diff_g);

  P_ = P_ - Kg*S_radar*Kg.transpose();

  NIS_radar_ = (z_diff_g).transpose() * Si * (z_diff_g);
}
/* EOF */