#pragma once
#include <Eigen/Dense>

class KalmanFilter
{
public:
  using Vector = Eigen::VectorXd;
  using Matrix = Eigen::MatrixXd;

  KalmanFilter(int nx, int nm);         // nx = state dim, nm = measurement dim
  void setModel(const Matrix& A, const Matrix& B,
                const Matrix& H,
                const Matrix& Q, const Matrix& R);

  void init(const Vector& x0, const Matrix& P0);

  void predict();                       // No control input 
  void predict(const Vector& u);        // With control input
   
  void update(const Vector& z);

  const Vector& state()  const { return x_; }
  const Matrix& cov()    const { return P_; }

private:
  int nx_, nm_;
  Matrix A_, B_, H_, Q_, R_, P_;
  Vector x_;
};
