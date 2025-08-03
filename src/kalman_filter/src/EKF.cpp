#include "EKF.hpp"


ExtendedKalmanFilter::ExtendedKalmanFilter(int nx, int nm)
: nx_(nx), nm_(nm),
  Q_(Matrix::Identity(nx,nx) * 1e-3),
  R_(Matrix::Identity(nm,nm) * 1e-2),
  P_(Matrix::Identity(nx,nx)),
  x_(Vector::Zero(nx))
{}

void ExtendedKalmanFilter::setModel(SystemFunc f, MeasFunc h,
                                    SystemJac F, MeasJac H,
                                    const Matrix& Q, const Matrix& R)
{
  f_ = f; h_ = h; F_ = F; H_ = H;
  Q_ = Q; R_ = R;
}

void ExtendedKalmanFilter::init(const Vector& x0, const Matrix& P0)
{
  x_ = x0;
  P_ = P0;
}


void ExtendedKalmanFilter::predict(const Vector& u)
{
    x_ = f_(x_, u);                                            // State transition function
    P_ = F_(x_, u) * P_ * F_(x_, u).transpose() + Q_;          // Jacobian of f
}

void ExtendedKalmanFilter::update(const Vector& z)
{
    Vector y = z - h_(x_);                                     // Innovation
    Matrix S = H_(x_) * P_ * H_(x_).transpose() + R_;          // Innovation covariance
    Matrix K = P_ * H_(x_).transpose() * S.inverse();          // Kalman gain
    x_ += K * y;                                               // Update state
    P_ = (Matrix::Identity(nx_, nx_) - K * H_(x_)) * P_;       // Update covariance   
}
