#pragma once

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace plvins {
inline int64_t nanoseconds(const builtin_interfaces::msg::Time& t) {
  return int64_t(t.sec) * 1000000000LL + t.nanosec;
}
inline double seconds(const builtin_interfaces::msg::Time& t) {
  return double(t.sec) + double(t.nanosec) * 1e-9;
}
inline builtin_interfaces::msg::Time stamp(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0 || seconds >= 2147483648.0)
    throw std::invalid_argument("timestamp outside ROS message range");
  return rclcpp::Time(int64_t(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}
template<class T>
bool parameter(const rclcpp::Node::SharedPtr& node, const std::string& name, T& value) {
  if (!node->has_parameter(name)) node->declare_parameter<T>(name, value);
  value = node->get_parameter(name).get_value<T>();
  return true;
}
inline std::string output_directory(const rclcpp::Node::SharedPtr& node) {
  std::string result = "results";
  parameter(node, "output_directory", result);
  auto path = std::filesystem::absolute(result);
  std::filesystem::create_directories(path);
  return path.string();
}
}  // namespace plvins

#define PLVINS_ASSERT(condition) do { if (!(condition)) \
  throw std::runtime_error("assertion failed: " #condition); } while (false)
