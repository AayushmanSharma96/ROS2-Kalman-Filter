#include "KalmanFilter.hpp"
#include <cassert>

KalmanFilter::KalmanFilter(int nx, int nm)
: nx_(nx), nm_(nm),
  A_(Matrix::Identity(nx,nx)),          // State transition matrix
  B_(Matrix::Zero(nx, nx)),             // Control input matrix (not used here)
  H_(Matrix::Zero(nm, nx)),             // Measurement matrix
  Q_(Matrix::Identity(nx,nx) * 1e-3),   // Process noise covariance
  R_(Matrix::Identity(nm,nm) * 1e-2),   // Measurement noise covariance
  P_(Matrix::Identity(nx,nx)),          // Initial covariance
  x_(Vector::Zero(nx))                  // Initial state vector
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
  /* no control input */
  x_ = A_ * x_;
  P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::predict(const Vector& u)
{
  /* with control input */
  assert(u.size() == B_.cols() && "u has wrong dimension");
  x_ = A_ * x_ + B_ * u;
  P_ = A_ * P_ * A_.transpose() + Q_;
}

void KalmanFilter::update(const Vector& z)
{
  Vector y  = z - H_ * x_;                                // Innovation
  Matrix S  = H_ * P_ * H_.transpose() + R_;              // Innovation covariance
  Matrix K  = P_ * H_.transpose() * S.inverse();          // Kalman gain

  x_ += K * y;                                            // Update state
  P_  = (Matrix::Identity(nx_, nx_) - K * H_) * P_;       // Update covariance
}
