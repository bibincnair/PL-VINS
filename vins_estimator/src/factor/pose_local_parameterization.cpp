#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double* x, const double* delta, double* out) const {
  const Eigen::Map<const Eigen::Vector3d> p(x), dp(delta);
  const Eigen::Map<const Eigen::Quaterniond> q(x + 3);
  Eigen::Map<Eigen::Vector3d> result_p(out);
  Eigen::Map<Eigen::Quaterniond> result_q(out + 3);
  // Preserve the original normalized small-angle retraction exactly.
  Eigen::Quaterniond dq(1., delta[3] * .5, delta[4] * .5, delta[5] * .5);
  result_p = p + dp;
  result_q = (q * dq).normalized();
  return true;
}
bool PoseLocalParameterization::PlusJacobian(const double* x, double* data) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(data);
  const Eigen::Map<const Eigen::Quaterniond> q(x + 3);
  const Eigen::Vector3d v = q.vec();
  Eigen::Matrix3d skew;
  skew << 0., -v.z(), v.y(), v.z(), 0., -v.x(), -v.y(), v.x(), 0.;
  J.setZero(); J.topLeftCorner<3,3>().setIdentity();
  J.block<3,3>(3,3) = .5 * (q.w() * Eigen::Matrix3d::Identity() + skew);
  J.block<1,3>(6,3) = -.5 * v.transpose();
  return true;
}
bool PoseLocalParameterization::Minus(const double* y, const double* x, double* data) const {
  Eigen::Map<Eigen::Matrix<double,6,1>> d(data);
  d.head<3>() = Eigen::Map<const Eigen::Vector3d>(y) - Eigen::Map<const Eigen::Vector3d>(x);
  Eigen::Quaterniond dq = Eigen::Map<const Eigen::Quaterniond>(x+3).conjugate() * Eigen::Map<const Eigen::Quaterniond>(y+3);
  if (std::abs(dq.w()) < 1e-12) return false;
  d.tail<3>() = 2. * dq.vec() / dq.w();
  return true;
}
bool PoseLocalParameterization::MinusJacobian(const double* x, double* data) const {
  Eigen::Matrix<double,7,6,Eigen::RowMajor> plus;
  PlusJacobian(x, plus.data());
  Eigen::Map<Eigen::Matrix<double,6,7,Eigen::RowMajor>> J(data);
  J = plus.transpose(); J.bottomRows<3>() *= 4.;
  return true;
}
