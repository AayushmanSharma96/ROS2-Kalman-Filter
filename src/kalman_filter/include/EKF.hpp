#pragma once
#include <Eigen/Dense>
#include <functional>

class ExtendedKalmanFilter
{
public:
  using Vector = Eigen::VectorXd;
  using Matrix = Eigen::MatrixXd;
  using SystemFunc = std::function<Vector(const Vector&, const Vector&)>;
  using SystemJac  = std::function<Matrix(const Vector&, const Vector&)>;
  using MeasFunc = std::function<Vector(const Vector&)>;
  using MeasJac    = std::function<Matrix(const Vector&)>;
 

  ExtendedKalmanFilter(int nx, int nm);
  void setModel(SystemFunc f, MeasFunc h,
                SystemJac F, MeasJac H,
                const Matrix& Q, const Matrix& R);

  void init(const Vector& x0, const Matrix& P0);

  void predict(const Vector& u);     
  void update(const Vector& z);

  const Vector& state()  const { return x_; }
  const Matrix& cov()    const { return P_; }

private:
  int nx_, nm_;
  SystemFunc f_;                     // State transition function
  MeasFunc h_;                       // Measurement function
  SystemJac F_;                      // Jacobian of f
  MeasJac H_;                        // Jacobian of h
  Matrix Q_, R_, P_;                 // Process noise, measurement noise, covariance
  Vector x_;                         // State vector
};
