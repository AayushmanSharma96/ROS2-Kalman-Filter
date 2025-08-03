#pragma once
#include <Eigen/Dense>

class KalmanFilter
{
public:
  using Vector = Eigen::VectorXd;
  using Matrix = Eigen::MatrixXd;

  KalmanFilter(int n, int m);                 // n = state dim, m = meas dim
  void setModel(const Matrix& A, const Matrix& B,
                const Matrix& H,
                const Matrix& Q, const Matrix& R);

  void init(const Vector& x0, const Matrix& P0);

  void predict();                       // 0-arg version
  void predict(const Vector& u);        // 1-arg version  ← add this line
   
  void update(const Vector& z);

  const Vector& state()  const { return x_; }
  const Matrix& cov()    const { return P_; }

private:
  int n_, m_;
  Matrix A_, B_, H_, Q_, R_, P_;
  Vector x_;
};
