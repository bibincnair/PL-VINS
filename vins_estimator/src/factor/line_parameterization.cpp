#include "line_parameterization.h"
#include <Eigen/Geometry>
#include <cmath>
namespace {
Eigen::Matrix3d rotation(const double* x) {
  return (Eigen::AngleAxisd(x[2], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(x[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(x[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
}
double wrap(double x) { return std::atan2(std::sin(x), std::cos(x)); }
}
bool LineOrthParameterization::Plus(const double* x, const double* d, double* out) const {
  Eigen::Matrix3d R = rotation(x) *
    (Eigen::AngleAxisd(d[0], Eigen::Vector3d::UnitX()) *
     Eigen::AngleAxisd(d[1], Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(d[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
  out[0] = std::atan2(R(2,1), R(2,2));
  out[1] = std::atan2(-R(2,0), std::hypot(R(0,0), R(1,0)));
  out[2] = std::atan2(R(1,0), R(0,0));
  out[3] = wrap(x[3] + d[3]);
  return true;
}
bool LineOrthParameterization::PlusJacobian(const double* x, double* data) const {
  if (std::abs(std::cos(x[1])) < 1e-10) return false;
  Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> J(data);
  const double s=std::sin(x[0]), c=std::cos(x[0]), t=std::tan(x[1]), cy=std::cos(x[1]);
  J << 1., s*t, c*t, 0., 0., c, -s, 0., 0., s/cy, c/cy, 0., 0., 0., 0., 1.;
  return true;
}
bool LineOrthParameterization::Minus(const double* y, const double* x, double* out) const {
  const Eigen::Matrix3d D = rotation(x).transpose() * rotation(y);
  out[0] = std::atan2(-D(1,2), D(2,2));
  out[1] = std::atan2(D(0,2), std::hypot(D(0,0), D(0,1)));
  out[2] = std::atan2(-D(0,1), D(0,0));
  out[3] = wrap(y[3] - x[3]);
  return true;
}
bool LineOrthParameterization::MinusJacobian(const double* x, double* data) const {
  Eigen::Matrix<double,4,4,Eigen::RowMajor> J;
  if (!PlusJacobian(x, J.data())) return false;
  Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> result(data);
  result = J.inverse();
  return true;
}
