#include "KalmanFilter.hpp"
#include <cassert>

KalmanFilter::KalmanFilter(int n, int m)
: n_(n), m_(m),
  A_(Matrix::Identity(n,n)),
  B_(Matrix::Zero(n, n)),
  H_(Matrix::Zero(m, n)),
  Q_(Matrix::Identity(n,n) * 1e-3),
  R_(Matrix::Identity(m,m) * 1e-2),
  P_(Matrix::Identity(n,n)),
  x_(Vector::Zero(n))
{}

void KalmanFilter::setModel(const Matrix& A, const Matrix& B,
                            const Matrix& H,
                            const Matrix& Q, const Matrix& R)
{
  A_ = A; B_ = B; H_ = H; Q_ = Q; R_ = R;
}

void KalmanFilter::init(const Vector& x0, const Matrix& P0)
{
  x_ = x0;
  P_ = P0;
}

void KalmanFilter::predict()
{
  // no control input
  x_ = A_ * x_;
  P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::predict(const Vector& u)
{
  assert(u.size() == B_.cols() && "u has wrong dimension");
  x_ = A_ * x_ + B_ * u;
  P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Vector& z)
{
  auto y  = z - H_ * x_;                       // innovation
  auto S  = H_ * P_ * H_.transpose() + R_;     // innovation cov
  auto K  = P_ * H_.transpose() * S.inverse(); // Kalman gain

  x_ += K * y;
  P_  = (Matrix::Identity(n_, n_) - K * H_) * P_;
}
