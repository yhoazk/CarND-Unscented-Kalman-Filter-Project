#include <iostream>
#include "ukf.h"

UKF::UKF() {
  //TODO Auto-generated constructor stub
  Init();
}

UKF::~UKF() {
  //TODO Auto-generated destructor stub
}

void UKF::Init() {

}


/*******************************************************************************
* Programming assignment functions:
*******************************************************************************/

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);

  //create vector for predicted state
  VectorXd x = VectorXd(n_x);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x, n_x);


/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //set weights
  weights(0)  = lambda / (lambda + n_aug);
  /* Set w for each column i.e. each of the sigma points */
  for (size_t i = 1; i < 2*n_aug+1; i++)
  {
    weights(i) = 1 / (2*(lambda + n_aug));
  }
  //predict state mean
  for (size_t i = 0; i < 2*n_aug+1; i++) {
    x = x+ weights(i) * Xsig_pred.col(i);
  }
  //predict state covariance matrix
  for (size_t i = 0; i < 2*n_aug+1  ; i++) {
      VectorXd x_diff = Xsig_pred.col(i) - x;
      //angle normalization
      // set the angle between +Pi and -Pi
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

      P +=  weights(i)*x_diff*x_diff.transpose();
  }


/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
}


/*
Expected:
x =
5.93637
1.49035
2.20528
0.536853
0.353577

P =

 0.00543425  -0.0024053   0.00341576  -0.00348196   -0.00299378
-0.0024053    0.010845    0.0014923    0.00980182    0.00791091
 0.00341576   0.0014923   0.00580129   0.000778632   0.000792973
-0.00348196   0.00980182  0.000778632  0.0119238     0.0112491
-0.00299378   0.00791091  0.000792973  0.0112491     0.0126972


*/
