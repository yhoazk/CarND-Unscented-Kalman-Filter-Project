#include "ukf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd::Zero(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5; // 5 state dimensions
  Xsig_pred_ = MatrixXd::Zero(n_x_ + 2,2*n_x_ + 1); // TODO: Dimensions
  lambda_ = 3 - n_x_;

  /* Covariance matrix */
  // TODO: Inital values?
  P_ = MatrixXd::Zero(n_x_, n_x_);

  Xsig_aug_ = MatrixXd::Zero(7,7);
  x_aug_.fill(0.0);
  S_laser = MatrixXd(nlaser_z, nlaser_z);
  S_radar = MatrixXd(nradar_z, nradar_z);
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
    std::cout << "------" << '\n';
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
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //create augmented covariance matrix
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * L.col(i);
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
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
    }
    else {
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

  return (status);
}
/**
 * @returns bool True if there were no errors false otherwise
 */
bool UKF::PredictMeanAndCovariance(void)
{
  bool status = true;
  //set weights
  weights_(0)  = lambda_ / (lambda_ + n_aug_);
  /* Set w for each column i.e. each of the sigma points */
  for (size_t i = 1; i < 2*n_aug_+1; i++)
  {
    weights_(i) = 1 / (2*(lambda_ + n_aug_));
  }
  //predict state mean
  for (size_t i = 0; i < 2*n_aug_+1; i++) {
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }
  //predict state covariance matrix
  for (size_t i = 0; i < 2*n_aug_+1  ; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    // set the angle between +Pi and -Pi
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ +=  weights_(i)*x_diff*x_diff.transpose();
  }

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
  rho = sqrt(state(0) *state(0) + state(1)*state(1) );
  phi = atan2(state(1) , state(0));
  rho_dot = (state(0) * cos(state(3))*state(2) + \
                state(1) * sin(state(3))*state(2)) / rho;

  VectorXd ret_vec(3);
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
  dt = (meas_package.timestamp_ - previous_timestamp_)/ (double_t)1000000.0;
  if(!is_initialized_)
  {
    P_ <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
        -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
        0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
        -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
        -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
    if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ = meas_package.raw_measurements_;
      std::cout << x_ << std::endl;
    } else /* Measurement is radar */
    {
      //x_.fill(0.0f);

      double_t px= meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      double_t py = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
      if(fabs(px) <= 0.001f || fabs(py) <= 0.001f)
      {
        return;
      }
      x_ << px,py,0,0,0;
    }

    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }


  if(meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UKF::UpdateLidar(meas_package);
  } else /* Is radar */
  {
    UKF::UpdateRadar(meas_package);
  }

  UKF::Prediction(dt);
  previous_timestamp_ = meas_package.timestamp_;
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
  MatrixXd Si = S_radar.inverse();

  //calculate cross correlation matrix
  for (size_t i = 0; i < 2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd Kg = Tc * Si;
  //update state mean and covariance matrix
  x_ = x_ + Kg*(meas_package.raw_measurements_ - z_pred);

  P_ = P_ - Kg*S*Kg.transpose();

  NIS_radar_ = (meas_package.raw_measurements_ - z_pred).transpose() * Si * (meas_package.raw_measurements_ - z_pred);
}
