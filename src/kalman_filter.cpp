#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // predict the state
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  //update the state by using Kalman Filter equations
  VectorXd y_ = z - H_ * x_;
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  //new estimate
  x_ = x_ + (K_ * y_);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //update the state by using Extended Kalman Filter equations
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  VectorXd hx(3);
  hx<< 0,0,0;
  hx(0) = max(0.0001, sqrt(pow(px,2) + pow(py,2)));
  hx(1) = atan2(py, px);
  hx(2) = (px * vx+ py * vy)/hx(0);

  //float ro = sqrt(px * px + py * py);
  //float theta = atan2(py, px);
  //float ro_dot;

  //if(fabs(ro)< 0.0001){
  //  ro_dot = 0;}  
  //else {
  //  ro_dot = (px * vx + py * vy)/ro;
  //}
 // VectorXd hx(3);
  //hx <<ro, theta, ro_dot;

  VectorXd y_ = z - hx;
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();


  //new estimate
  x_ = x_ + (K_ * y_);
  long x_size = x_.size();
  MatrixXd I = MatrixXd:: Identity(x_size, x_size);
  P_ = (I - K_ * H_) * P_;
}
