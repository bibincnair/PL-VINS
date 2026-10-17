#pragma once
#include <ceres/ceres.h>
#include <Eigen/Dense>

class LineOrthParameterization final : public ceres::Manifold {
 public:
  int AmbientSize() const override { return 4; }
  int TangentSize() const override { return 4; }
  bool Plus(const double*, const double*, double*) const override;
  bool PlusJacobian(const double*, double*) const override;
  bool Minus(const double*, const double*, double*) const override;
  bool MinusJacobian(const double*, double*) const override;
};
inline void liftLineJacobian(const double* x, double* data, int rows) {
  Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,4,Eigen::RowMajor>> J(data, rows, 4);
  Eigen::Matrix<double,4,4,Eigen::RowMajor> inverse;
  LineOrthParameterization().MinusJacobian(x, inverse.data());
  J = (J * inverse).eval();
}
