/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {
  int n = 4; // Number of states
  int m = 2; // Number of measurements

  double dt = double(1)/double(30); // Time step
  // Define system matrices
  Eigen::MatrixXd Ain(n, n); // System dynamics matrix
  Eigen::MatrixXd Cin(m, n); // Output matrix
  Eigen::MatrixXd Qin(n, n); // Process noise covariance
  Eigen::MatrixXd Rin(m, m); // Measurement noise covariance
  Eigen::MatrixXd Pin(n, n); // Estimate error covariance
  // Discrete LTI projectile motion, measuring position only
  Ain << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;
  Cin << 1, 0, 0, 0, 0, 1, 0, 0;
  // Covariance matrices
  double nPos = 1e-1;  // variance of spatial process noise
  double nVel = 1e-2;  // variance of velocity process noise
  double nMeas = 0.5;  // variance of measurement noise for z_x and z_y
  Qin << nPos, 0, 0, 0, 0, nPos, 0, 0, 0, 0, nVel, 0, 0, 0, 0, nVel;
  Rin << nMeas, 0, 0, nMeas;
  Pin << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  A = Ain; C = Cin; Q = Qin; R = Rin; P0 = Pin;

  // Initialize other parameters
  m = C.rows();
  n = A.rows();
  initialized = false;
  Eigen::MatrixXd eigenM(n,n);
  Eigen::VectorXd V(n);
  I = eigenM; x_hat = V; x_hat_new = V;
  I.setIdentity();

    // Show matrices
  if (DEBUG){
    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P0: \n" << P0 << std::endl;
    std::cout<< "I: \n" << I << std::endl;
  }
}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

  this->A = A;
  this->dt = dt;
  update(y);
}
