#pragma once
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Pose storage is [p, qx, qy, qz, qw], with right rotation increments.
class PoseLocalParameterization final : public ceres::Manifold {
 public:
  int AmbientSize() const override { return 7; }
  int TangentSize() const override { return 6; }
  bool Plus(const double*, const double*, double*) const override;
  bool PlusJacobian(const double*, double*) const override;
  bool Minus(const double*, const double*, double*) const override;
  bool MinusJacobian(const double*, double*) const override;
};

// Existing factor algebra produces tangent derivatives. Ceres requires ambient ones.
inline void liftPoseJacobian(const double* x, double* data, int rows) {
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 7, Eigen::RowMajor>> J(data, rows, 7);
  Eigen::Matrix<double, 6, 7, Eigen::RowMajor> inverse;
  PoseLocalParameterization().MinusJacobian(x, inverse.data());
  const Eigen::MatrixXd local = J.leftCols(6);
  J = local * inverse;
}
